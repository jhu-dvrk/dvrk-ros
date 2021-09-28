# Author: Jintan Zhang
# Date: 09/24/2021

import sys
import dvrk
import PyKDL
import numpy as np
import time as t
import rospy
import collections
from geometry_msgs.msg import WrenchStamped


class AtiRegister:
    def __init__(self, arm, iters, force, size):
        # create arm
        self.p = dvrk.psm(arm)

        # parameters
        self.order_name = ['x', 'y', 'z']
        self.order = [0, 1, 2]
        self.force = force
        self.iters = iters
        self.size = size
        self.use_lsq = False

        # rostopic subscriber
        self.ati_data = collections.deque(maxlen=size)

        rospy.Subscriber('/measured_cf', WrenchStamped, self.ati_cb)

        # test force
        self.jaw_cf = np.zeros((1, 1))

        # local ram
        self.register_out = []
        self.rot = []

    def ati_cb(self, msg):
        self.ati_data.append([msg.wrench.force.x] + [msg.wrench.force.y] + [msg.wrench.force.z])

    def reset_data(self):
        self.ati_data.clear()
        while len(self.ati_data) < self.size:
            continue
        return np.mean(self.ati_data, axis=0)

    def cal_rot(self, din, dout):
        assert din.shape[0] == dout.shape[0], "input force and feedback force size mismatch"

        n = din.shape[0]
        A = np.zeros((2 * n, 9))
        count = 0

        for i in range(2 * n):
            if i % 2 == 0:
                A[i, 0] = din[count][0]
                A[i, 1] = din[count][1]
                A[i, 2] = 1
                A[i, 3:6] = 0
                A[i, 6] = -dout[count][0] * din[count][0]
                A[i, 7] = -dout[count][0] * din[count][1]
                A[i, 8] = -dout[count][0]
            elif i % 2 == 1:
                A[i, 0:3] = 0
                A[i, 3] = din[count][0]
                A[i, 4] = din[count][1]
                A[i, 5] = 1
                A[i, 6] = -dout[count][1] * din[count][0]
                A[i, 7] = -dout[count][1] * din[count][1]
                A[i, 8] = -dout[count][1]
                count = count + 1

        u, d, vh = np.linalg.svd(A, full_matrices=True)
        t = (vh[-1, :] / vh[-1, -1]).reshape(3, 3)

        print("The homograpy is :\n{}".format(t))

        return t

    def register(self):
        # send zero force, user find contact point
        raw_input("Press any key to disable PID")
        body_cf = np.zeros((1, 6))
        self.p.jaw.servo_jf(body_cf)

        # contact point found, close grip
        raw_input("Press any key to close gripper")
        self.jaw_cf[0] = -0.15
        self.p.jaw.servo_jf(self.jaw_cf)

        # main loop
        r = rospy.Rate(100)

        raw_input("Press any key to start registration... (P.S.: might be a good idea to rebias sensor)")

        while not rospy.is_shutdown():

            for idx, j in enumerate(self.order):
                # exert force in x axis
                # raw_input("Press any key to exert {} force".format(self.order_name[idx]))
                print("exert {} force...".format(self.order_name[idx]))

                fb = []
                ref = []
                raw = []
                for i in range(iters):
                    print("Attempt {} ...".format(i))
                    # collect reference
                    ref.append(self.reset_data())

                    # apply force
                    if self.order_name[j] == 'x':
                        body_cf = np.array([self.force, 0, 0, 0, 0, 0])
                    elif self.order_name[j] == 'y':
                        body_cf = np.array([0, self.force, 0, 0, 0, 0])
                    elif self.order_name[j] == 'z':
                        body_cf = np.array([0, 0, self.force, 0, 0, 0])

                    self.p.body.servo_cf(body_cf)
                    t.sleep(2)

                    # collect feedback
                    fb.append(self.reset_data())
                    raw.append(body_cf)

                    # restore
                    self.p.body.servo_cf(-body_cf)
                    t.sleep(2)

                # compute rotation matrix (homography transform) using least square
                if self.use_lsq:
                    rot = self.cal_rot(np.array(raw), np.array(fb))
                    self.rot.append(rot)

                # compute mean
                # REMINDER: PSM X is ATI Y in current setup
                x_mean = np.mean(np.array(fb)[:, 1]) - np.mean(np.array(ref)[:, 1])
                y_mean = np.mean(np.array(fb)[:, 0]) - np.mean(np.array(ref)[:, 0])
                z_mean = np.mean(np.array(fb)[:, 2]) - np.mean(np.array(ref)[:, 2])
                print("registered {} vector = {}".format(self.order_name[j], (x_mean, y_mean, z_mean)))
                self.register_out.append(np.array([x_mean, y_mean, z_mean]))

            if len(self.register_out) == len(self.order_name):
                # --------------------------------------------------
                # check if vectors are perpendicular to each other
                # --------------------------------------------------
                print("sanity check...")
                vec_x = self.register_out[1]
                vec_y = self.register_out[0]
                vec_z = self.register_out[2]
                xy_deg = np.arccos(vec_x.dot(vec_y.T) / (np.linalg.norm(vec_x) * np.linalg.norm(vec_y))) * 180 / np.pi
                yz_deg = np.arccos(vec_y.dot(vec_z.T) / (np.linalg.norm(vec_y) * np.linalg.norm(vec_z))) * 180 / np.pi
                xz_deg = np.arccos(vec_x.dot(vec_z.T) / (np.linalg.norm(vec_x) * np.linalg.norm(vec_z))) * 180 / np.pi
                print("xy = {}, yz = {}, xz = {}".format(xy_deg, yz_deg, xz_deg))

                # --------------------------------------------------
                # Get transformation using svd
                # --------------------------------------------------
                Rot_M = np.array([[vec_x[0], vec_y[0], vec_z[0]],
                                  [vec_x[1], vec_y[1], vec_z[1]],
                                  [vec_x[2], vec_y[2], vec_z[2]]])
                u, s, vt = np.linalg.svd(Rot_M)
                Rot_M = u.dot(vt.T)

                # --------------------------------------------------
                # check correctness of calculated rotation matrix
                # --------------------------------------------------
                body_cf = np.array([1, 1, 1, 0, 0, 0])
                self.p.body.servo_cf(body_cf)
                t.sleep(2)

                # collect feedback
                test_gt = self.reset_data()

                # restore
                self.p.body.servo_cf(-body_cf)
                t.sleep(2)

                # check
                test_est = Rot_M.dot(body_cf[0:3].T)
                print("GT = {}, EST = {}".format(test_gt, test_est))

            decision = raw_input("Press r/q to repeat/exit...\n")

            if decision == 'q':
                sys.exit(0)

            r.sleep()


if __name__ == "__main__":
    arm = 'PSM2'
    iters = 25
    force = 2.5
    size = 100

    atiRegister = AtiRegister(arm, iters, force, size)
    atiRegister.register()
