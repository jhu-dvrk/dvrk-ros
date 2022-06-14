#!/usr/bin/env python

import collections
import cv2
import math
import numpy as np


# Represents a single tracked detection, with location history information
class TrackedObject:
    max_strength = 10

    def __init__(self, detection, history_length):
        # Last known position
        self.position = (detection[0], detection[1])
        self.size = detection[2]

        # 'signal-strength' i.e. how long and consistently this object has been detected for
        self.strength = 1

        # queue will 'remember' last history_length object locations
        self.location_history = collections.deque(maxlen=history_length)
        self.location_history.append(self.position)

    # Manhattan/L1 distance between this object and 'position'
    def distanceTo(self, position):
        dx = self.position[0] - position[0]
        dy = self.position[1] - position[1]
        return math.sqrt(dx*dx + dy*dy)

    def is_strong(self):
        return self.strength > (TrackedObject.max_strength - 2)

    # when a match is found, known position is updated and strength increased by 1,
    # if no match is found, strength decays by 2 to prevent strength from oscillating
    def update(self, detection):
        if detection is not None:
            self.position = (detection[0], detection[1])
            self.size = detection[2]
            self.location_history.append(self.position)

            # cap strength so objects never remain too long
            self.strength = min(self.strength+1, TrackedObject.max_strength)
        else:
            self.strength -= 2


# Track all detected objects as they move over time
class ObjectTracking:
    # max_distance is how far objects can move between frames
    # and still be considered the same object
    def __init__(self, max_distance, history_length):
        self.objects = []
        self.max_distance = max_distance
        self.primaryTarget = None
        self.history_length = history_length

    # mark on object as the primary object to track
    def setPrimaryTarget(self, position):
        nearbyObjects = [x for x in self.objects if x.distanceTo(position) < x.size]
        self.primaryTarget = nearbyObjects[0] if len(nearbyObjects) == 1 else None
        if self.primaryTarget is not None:
            self.primaryTarget.location_history.clear()

    # Clear tracked objects
    def clear(self):
        self.objects = []
        self.primaryTarget = None

    def clear_history(self):
        for obj in self.objects:
            obj.location_history.clear()

    # register detections from the current frame with tracked objects,
    # removing, updating, and adding tracked objects as needed
    def register(self, detections):
        distances = np.array([
            np.array([obj.distanceTo(d) for d in detections])
            for obj in self.objects
        ])

        current_object_count = len(self.objects)
        
        if len(self.objects) > 0:
            if len(detections) > 0:
                closest = np.argmin(distances, axis=0)

                # associate detections with existing tracked objects or add as new objects
                for i, detection in enumerate(detections):
                    if distances[closest[i], i] <= self.max_distance:
                        self.objects[closest[i]].update(detection)
                    else:
                        self.objects.append(TrackedObject(detection, history_length=self.history_length))

                # Update tracked objects not associated with current detection
                closest = np.argmin(distances, axis=1)
                for j in range(current_object_count):
                    if distances[j, closest[j]] > self.max_distance:
                        self.objects[j].update(None)
            else:
                for obj in self.objects:
                    obj.update(None)

            # Remove stale tracked objects and check if primary target is stale
            self.objects = [x for x in self.objects if x.strength > 0]
            if not self.primaryTarget in self.objects and self.primaryTarget is not None:
                print("Lost track of target! Please click on target to re-acquire")
                self.primaryTarget = None

        # No existing tracked objects, add all detections as new objects
        else:
            for d in detections:
                self.objects.append(TrackedObject(d, history_length=self.history_length)) 


# tracks current offset of remote-center-of-motion of PSM
class RCMTracker:
    def __init__(self, expected_motion_angle, tracking_distance=12, window_title="CV Calibration"):
        self.objects = ObjectTracking(tracking_distance, history_length=200)
        self.window_title = window_title

        self.arm_swing_angle = expected_motion_angle


    def _mouse_callback(self, event, x, y, flags, params):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.objects.setPrimaryTarget((x, y))
    

    def _create_window(self):
        cv2.namedWindow(self.window_title)
        cv2.setMouseCallback(self.window_title, lambda *args: self._mouse_callback(*args))


    def _init_video(self):
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(cv2.CAP_PROP_FPS, 30)        
        ok = False
        if self.video_capture.isOpened():
            ok, _ = self.video_capture.read()
        
        if not ok:
            print("Couldn't read from camera.")

        return ok


    def _release(self):
        self.video_capture.release()
        cv2.destroyWindow(self.window_title)


    def _process(self, frame):
        blurred = cv2.medianBlur(frame, 2*5 + 1)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv, (135, int(35*2.55), int(25*2.55)), (180, int(100*2.55), int(75*2.55)))
        
        contours, _ = cv2.findContours(
            thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        contours = [c for c in contours if cv2.contourArea(c) > 15]

        #cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        moments = [cv2.moments(c) for c in contours]
        radius = lambda area: math.sqrt(area/math.pi)
        detections = [(int(M['m10']/M['m00']), int(M['m01']/M['m00']), radius(M['m00'])) for M in moments if M['m00'] > 0]

        self.objects.register(detections)

        for c in contours:
            M = cv2.moments(c)
            center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))
            if self.objects.primaryTarget is None or self.objects.primaryTarget.position[0] != center[0] or self.objects.primaryTarget.position[1] != center[1]:
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            elif self.objects.primaryTarget is not None:
                cv2.drawContours(frame, [c], -1, (255, 0, 255), 5)    

    
    def is_good_ellipse_fit(self, target_bound, ellipse_bound):
        rect_area = lambda rect: rect[1][0]*rect[1][1]
        target_area = rect_area(target_bound)
        if target_area <= 0.0:
            return False
        
        # theoretical ratio between bounding rect area of observed target trajectory and
        # bounding rect area of full circle 
        expected_area_ratio = (1.0 - math.cos(0.5*self.arm_swing_angle))*(2*math.sin(0.5*self.arm_swing_angle))/4.0
        allowed_margin = 2.0

        # If the fitted ellipse is significantly larger than expected, there is likely outlier points
        # causing a poor fit
        if expected_area_ratio * rect_area(ellipse_bound) > allowed_margin * target_area:
            return False

        intersection_type, intersection_contour = cv2.rotatedRectangleIntersection(ellipse_bound, target_bound)
        overlap_ratio = 0.0
        if intersection_type != cv2.INTERSECT_NONE:
            overlap_ratio = cv2.contourArea(intersection_contour)/target_area

        # Bounding rects should have nearly 100% overlap, but poor fit can cause ellipse to point in the
        # wrong direction, etc. leading to smaller overlap. So this indicates bad fit
        if overlap_ratio < 0.85:
            return False
       
        return True


    def fit_ellipse(self, tracked_object):
        location_history = np.array(tracked_object.location_history)
        bounding_rect = cv2.minAreaRect(location_history)
        ellipse_bound = cv2.fitEllipse(location_history)
        
        # Sanity check quality of ellipse fitting
        if not self.is_good_ellipse_fit(bounding_rect, ellipse_bound):
            return None, None, ellipse_bound, bounding_rect

        ellipse_center, (width, height), _ = ellipse_bound
        ellipse_center = np.array(ellipse_center)
        location_history_center = bounding_rect[0] 
        rcm_offset = np.array(ellipse_center) - location_history_center
        radius = (width + height)/4.0

        return radius, rcm_offset, ellipse_bound, bounding_rect

    
    def init(self):
        self._create_window()
        ok = self._init_video()
        return ok


    def pause(self, paused):
        self.paused = paused


    def stop(self):
        self.should_stop = True


    def clear_history(self):
        self.objects.clear_history()


    def run_point_acquisition(self):
        key = None
        ok = self.init()
        self.objects.clear()
        self.paused = False
        self.should_stop = False

        while ok and not self.should_stop:
            ok, frame = self.video_capture.read()
            self._process(frame)
            cv2.imshow(self.window_title, frame)

            if not self.paused:
                target = self.objects.primaryTarget
                if target is not None and target.is_strong():
                    cv2.circle(frame, target.position, radius=3, color=(0, 0, 255), thickness=cv2.FILLED)
                    if len(target.location_history) > 5:
                        mean = np.mean(target.location_history, axis=0)
                        self._release()
                        return True, np.int0(mean) 

            key = cv2.waitKey(20)
            escape = 27
            if key == ord("q") or key == escape:
                break

        self._release()
        return False, None


    def run(self, output_callback):
        try:
            key = None
            ok = self.init()
            self.paused = False
            self.should_stop = False

            while ok and not self.should_stop:
                ok, frame = self.video_capture.read()
                self._process(frame)

                if not self.paused:
                    target = self.objects.primaryTarget
                    if target is not None and target.is_strong():
                        cv2.circle(frame, target.position, radius=3, color=(0, 0, 255), thickness=cv2.FILLED)
                        if len(target.location_history) > 5:
                            radius, rcm_offset, ellipse, rect = self.fit_ellipse(target)
                            cv2.ellipse(frame, ellipse, color=(255, 0, 0), thickness=1)
                            cv2.drawContours(frame, [np.int0(cv2.boxPoints(ellipse))], 0, color=(0, 255, 0), thickness=1)
                            cv2.drawContours(frame, [np.int0(cv2.boxPoints(rect))], 0, color=(0, 0, 255), thickness=1)
                            cv2.drawContours(frame, [np.int0(list(target.location_history))], 0, color=(255, 255, 0), thickness=1)

                            ellipse_center = (int(ellipse[0][0]), int(ellipse[0][1]))
                            location_history_mean = np.mean(target.location_history, axis=0)
                            location_history_center = (int(location_history_mean[0]), int(location_history_mean[1]))    
                            rect_center = (int(rect[0][0]), int(rect[0][1]))
                            
                            cv2.circle(frame, ellipse_center, radius=0, color=(0, 255, 0), thickness=3)
                            cv2.circle(frame, location_history_center, radius=0, color=(255, 0, 0), thickness=3)
                            cv2.circle(frame, rect_center, radius=0, color=(0, 0, 255), thickness=3)

                            if radius is not None:
                                if len(target.location_history) > 75:
                                    output_callback(rcm_offset, radius)

                cv2.imshow(self.window_title, frame)

                key = cv2.waitKey(20)
                escape = 27
                if key == ord("q") or key == escape:
                    break

        except KeyboardInterrupt:
            pass 
    
        finally:
            self._release()

        return True

