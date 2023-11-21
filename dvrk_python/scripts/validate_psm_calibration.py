#!/usr/bin/env python

import rospy
import argparse
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Bool
import numpy as np
import math
import os
import PyKDL
import sys
import time
import dvrk

import cisst_msgs.srv

import xml.etree.ElementTree as ET

# To use this script, run the sawNDITracker ROS tool with an arbitrary tool definition, and enable
# stray marker tracking. Also run the console for the relevant arm in calibration mode. Place
# a passive tracker sphere onto the end of a PSM tool - you may need to open the jaws somewhat to
# hold it in place. Ensure the PSMs working range is within the tracker's volume, and no
# other detections are picked up.

class NDITracker:
    def init(self):
        self.topic = "/NDI/measured_cp_array"
        self.subscriber = rospy.Subscriber(self.topic, PoseArray, self._marker_callback) 
        self.is_tracking_strays = rospy.Subscriber("/NDI/tracking/stray_markers", Bool, self._is_tracking_strays)
        self._last_position = None
        self.valid = False

    def _is_tracking_strays(self, bool_msg):
        if not bool_msg.data:
            print("Not tracking stray markers!") 

    def _marker_callback(self, pose_array_msg):
        marker_count = len(pose_array_msg.poses)
        self.valid = marker_count == 1
        if not self.valid: 
            return

        self._last_position = pose_array_msg.poses[0].position

    def get_position(self):
        return self.valid, self._last_position

    def stop(self):
        self.is_tracking_strays.unsubscribe()
        self.subscriber.unsubscribe()


class CalibrationValidationApplication:
    def configure(self, robot_name, config_file, expected_interval):
        # check that the config file is good
        if not os.path.exists(config_file):
            print('Config file "{:s}" not found'.format(config_file))
            return False

        self.tree = ET.parse(config_file)
        root = self.tree.getroot()

        xpath_search_results = root.findall("./Robot")
        if len(xpath_search_results) != 1:
            print('Can\'t find "Robot" in config file "{:s}"'.format(config_file))
            return False

        xmlRobot = xpath_search_results[0]
        # Verify robot name
        if xmlRobot.get("Name") != robot_name:
            print(
                'Found robot "{:s}" instead of "{:s}", are you using the right config file?'.format(
                    xmlRobot.get("Name"), robot_name
                )
            )
            return False

        serial_number = xmlRobot.get("SN")
        print(
            'Successfully found robot "{:s}", serial number {:s} in XML file'.format(
                robot_name, serial_number
            )
        )
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(arm_name=robot_name, expected_interval=expected_interval)

        self.cartesian_insertion_minimum = 0.055

        return True

    def setup(self):
        if not self.arm.enable(10):
            print("Failed to enable within 10 seconds")
            return False

        if not self.arm.home(10):
            print("Failed to home within 10 seconds")
            return False

        return True

    # instrument needs to be inserted past cannula to use Cartesian commands,
    # this will move instrument if necessary so Cartesian commands can be used
    def enter_cartesian_space(self):
        pose = np.copy(self.arm.measured_jp())
        if pose[2] >= self.cartesian_insertion_minimum:
            return True

        pose[2] = self.cartesian_insertion_minimum
        self.arm.move_jp(pose).wait()
        return True

    # return arm to upright position
    def center_arm(self):
        pose = np.copy(self.arm.measured_jp())
        pose.fill(0.0)
        pose[2] = self.cartesian_insertion_minimum
        self.arm.move_jp(pose).wait()
        return True

    # Generate series of arm poses within safe range of motion
    # range_of_motion = (depth, radius, center) describes a
    #     cone with tip at RCM, base centered at (center, depth)
    # Generates slices^3 poses total
    def registration_poses(self, slices=5, rom=math.pi/4, max_depth=0.018):
        query_cp_name = "{}/local/query_cp".format(self.arm.namespace()) 
        local_query_cp = rospy.ServiceProxy(query_cp_name, cisst_msgs.srv.QueryForwardKinematics)

        # Scale to keep point density equal as depth varies
        scale_rom = lambda depth: math.atan((max_depth/depth)*math.tan(rom))

        def merge_coordinates(alpha, betas, depth):
            alphas = np.repeat(alpha, slices)
            depths = np.repeat(depth, slices)
            return np.column_stack([alphas, betas, depths])

        js_points = []
        depths = np.linspace(max_depth, self.cartesian_insertion_minimum, slices)
        for i, depth in enumerate(depths):
            parity = 1 if i % 2 == 0 else -1
            theta = scale_rom(depth)*parity
            alphas = np.linspace(-theta, theta, slices)
            # Alternate direction so robot follows shortest path
            for i, alpha in enumerate(alphas):
                parity = 1 if i % 2 == 0 else -1
                betas = np.linspace(-parity*theta, parity*theta, slices)
                js_points.extend(merge_coordinates(alpha, betas, depth))

        # We generated square grid, crop to circle so that overall angle
        # stays within specified range of motion
        js_points = [p for p in js_points if (p[0]**2 + p[1]**2) <= rom**2]

        cs_points = []
        for point in js_points:
            # query forward kinematics to get equivalent Cartesian point
            kinematics_request = cisst_msgs.srv.QueryForwardKinematicsRequest()
            kinematics_request.jp.position = [point[0], point[1], point[2], 0.0, 0.0, 0.0, 0.0, 0.0]
            response = local_query_cp(kinematics_request)
            point = response.cp.pose.position
            cs_points.append(np.array([point.x, point.y, point.z]))

        goal_orientation = PyKDL.Rotation()
        goal_orientation[1,1] = -1.0
        goal_orientation[2,2] = -1.0

        points = [PyKDL.Vector(p[0], p[1], p[2]) for p in cs_points]
        poses = [PyKDL.Frame(goal_orientation, p) for p in points]

        return poses

    # move arm to each goal pose, and measure both robot and camera relative positions
    def collect_data(self, poses, tracker_sample_size=10):
        ok = self.enter_cartesian_space()
        if not ok:
            return False, None, None

        robot_points = []
        tracker_points = []

        print("Collecting data: 0/{}".format(len(poses)), end="\r")

        # Move arm to variety of positions and record image & world coordinates
        for i, pose in enumerate(poses):
            if not self.ok:
                self.center_arm()
                break

            self.arm.move_cp(pose).wait()
            rospy.sleep(0.5) # Make sure arm is settled
            
            point = self.arm.measured_cp().p
            point = np.array([point[0], point[1], point[2]])
            robot_points.append(point)

            tracker_sample = []

            for j in range(tracker_sample_size):
                rospy.sleep(0.05)
                if rospy.is_shutdown():
                    return False, None, None

                ok = False
                while not ok and not rospy.is_shutdown():
                    ok, point = self.tracker.get_position()

                tracker_sample.append(np.array([point.x, point.y, point.z]))

            tracker_points.append(np.mean(tracker_sample, axis=0))

            print(
                "Collecting data: {}/{}".format(i+1, len(poses)),
                end="\r",
            )

        print("\n")

        self.center_arm()
        return True, robot_points, tracker_points

    # Compute rigid registration of robot_points to tracker_points,
    # assuming correspondence robot_points[i] <-> tracker_points[i]
    # Returns status, RMS error, rotation, translation
    # Uses Kabsch method
    def compute_registration(self, robot_points, tracker_points):
        robot_points = np.array(robot_points, dtype=np.float32)
        tracker_points = np.array(tracker_points, dtype=np.float32)

        robot_points_centroid = np.mean(robot_points, axis=0)
        tracker_points_centroid = np.mean(tracker_points, axis=0)
        
        # Align centroids to remove translation
        A = robot_points - robot_points_centroid
        B = tracker_points - tracker_points_centroid
        covariance = np.matmul(np.transpose(A), B)
        u, s, vh = np.linalg.svd(covariance)
        d = math.copysign(1, np.linalg.det(np.matmul(u, vh)))
        C = np.diag([1, 1, d])
        R = np.matmul(u, np.matmul(C, vh))
        if np.linalg.det(R) < 0:
            print("Reflection found instead!")
   
        T = robot_points_centroid - np.matmul(R, tracker_points_centroid)
        projected_tracker_points = np.matmul(tracker_points, np.transpose(R)) + T
        pointwise_difference = robot_points - projected_tracker_points
        pointwise_error = [np.linalg.norm(d) for d in pointwise_difference]

        # Compute error metrics, in millimeters
        rmse = 1000.0*np.sqrt(np.mean([error**2 for error in pointwise_error]))
        maxe = 1000.0*np.max(pointwise_error)
        stde = 1000.0*np.std(pointwise_error)

        return self.ok, (rmse, maxe, stde), R, T

    def run(self, trials):
        try:
            self.ok = True

            self.tracker = NDITracker()
            self.tracker.init()

            errors = []

            times = []
        
            for i in range(trials):
                start = time.time()
                self.ok = self.setup()
                if not self.ok:
                    return

                poses = self.registration_poses(5, math.radians(50), 0.200)
                self.ok, robot_points, tracker_points = self.collect_data(poses)
                if not self.ok:
                    return

                self.ok, error, _, _ = self.compute_registration(robot_points, tracker_points)
                if not self.ok:
                    return
            
                errors.append(error)
                print("Trial {}/{} RMS error: {} {} {} millimeters".format(i+1, trials, error[0], error[1], error[2]))
                end = time.time()
                duration = end - start
                times.append(duration)
                if i+1 < trials:
                    avg = np.mean(times)
                    remaining = avg*(trials - (i+1))
                    print("Estimated time remaining: {} seconds".format(int(remaining)))

            print("\nAll errors:")
            for error in errors:
                print("{},{},{}".format(error[0], error[1], error[2]))

        finally:
            self.arm.unregister()


if __name__ == '__main__':
    # ros init node so we can use default ros arguments (e.g. __ns:= for namespace)
    rospy.init_node('dvrk_validate_psm_calibration', anonymous=True)
    # strip ros arguments
    argv = rospy.myargv(argv=sys.argv)

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-t', '--trials', type=int, default=1,
                        help = 'how many full trials to run, disabling arm between trials')
    parser.add_argument('-i', '--interval', type=float, default=0.02,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-c', '--config', type=str, required=True,
                        help = 'arm IO config file, i.e. something like sawRobotIO1394-xwz-12345.xml')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    application = CalibrationValidationApplication()
    application.configure(args.arm, args.config, args.interval)
    application.run(args.trials)

