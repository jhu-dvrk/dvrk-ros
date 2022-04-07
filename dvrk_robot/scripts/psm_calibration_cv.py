#!/usr/bin/env python

import collections
import cv2
import math
import numpy as np


# Represents single tracked detection, with location history information
class TrackedObject:
    def __init__(self, detection):
        self.position = (detection[0], detection[1])
        self.size = detection[2]
        self.strength = 1
        self.history = collections.deque(maxlen=200)
        self.history.append(self.position)

    def distanceTo(self, position):
        dx = self.position[0] - position[0]
        dy = self.position[1] - position[1]
        return dx*dx + dy*dy

    def match(self, detection):
        self.position = (detection[0], detection[1])
        self.history.append(self.position)
        self.size = detection[2]
        self.strength = min(self.strength+1, 20)

    def noMatch(self):
        self.strength -= 2


# Track detected objects as they move around
class MotionTracking:
    def __init__(self, min_distance):
        self.objects = []
        self.minimum = min_distance**2
        self.primaryTarget = None

    def updatePrimary(self, position):
        nearbyObjects = [x for x in self.objects if x.distanceTo(position) < self.minimum]
        self.primaryTarget = nearbyObjects[0] if len(nearbyObjects) == 1 else None
        if self.primaryTarget:
            self.primaryTarget.history.clear()

    def register(self, detections):
        distances = np.array([
            np.array([obj.distanceTo(d) for d in detections])
            for obj in self.objects
        ])

        current_object_count = len(self.objects)
        
        if len(self.objects) > 0:
            if len(detections) > 0:
                closest = np.argmin(distances, axis=0)

                for i, detection in enumerate(detections):
                    if distances[closest[i], i] <= self.minimum:
                        self.objects[closest[i]].match(detection)
                    else:
                        self.objects.append(TrackedObject(detection))

                closest = np.argmin(distances, axis=1)

                for j in range(current_object_count):
                    if distances[j, closest[j]] > self.minimum:
                        self.objects[j].noMatch()
            else:
                for obj in self.objects:
                    obj.noMatch()

            self.objects = [x for x in self.objects if x.strength > 0]
            self.primaryTarget = self.primaryTarget if self.primaryTarget in self.objects else None
        else:
            for d in detections:
                self.objects.append(TrackedObject(d)) 


class ObjectTracking:
    def __init__(self, tracking_distance=50, window_title="CV Calibration"):
        self.tracker = MotionTracking(tracking_distance)
        self.window_title = window_title

    def _mouse_callback(self, event, x, y, flags, params):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.tracker.updatePrimary((x, y))
    
    def _create_window(self):
        cv2.namedWindow(self.window_title)
        cv2.setMouseCallback(self.window_title, lambda *args: self._mouse_callback(*args))

    def _init_video(self):
        self.video_capture = cv2.VideoCapture(0)
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
        blurred = cv2.medianBlur(frame, 2*3 + 1)
        cv2.imwrite("capture.png", frame)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        thresholded = cv2.inRange(hsv, (135, int(35*2.55), int(25*2.55)), (180, int(100*2.55), int(75*2.55)))
        
        kernel_size = 2
        morph_element = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (2 * kernel_size + 1, 2 * kernel_size + 1),
            (kernel_size, kernel_size)
        )
       # image = cv2.erode(thresholded, morph_element)
       # image = cv2.dilate(image, morph_element)

        contours, hierarchies = cv2.findContours(
            thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        moments = [cv2.moments(c) for c in contours]
        detections = [(int(M['m10']/M['m00']), int(M['m01']/M['m00']), 5) for M in moments if M['m00'] != 0]

        self.tracker.register(detections)

    def run(self, output_callback):
        try:
            self._create_window()
            ok = self._init_video()
            if not ok:
                return False

            key = None

            while ok:
                ok, frame = self.video_capture.read()
                self._process(frame)

                target = self.tracker.primaryTarget
                if target is not None and target.strength >= 12:
                    cv2.circle(frame, target.position, radius=3, color=(0, 0, 255), thickness=cv2.FILLED)
                    if len(target.history) > 5:
                        history = np.array(target.history)
                        ellipse_bound = cv2.fitEllipse(history)
                        (ellipse_center, (width, height), angle) = ellipse_bound
                        radius = (width + height)/4.0
                        ellipse_center = np.array(ellipse_center)
                        history_center = np.mean(history)
                        angle = math.radians(angle)

                        cv2.ellipse(frame, ellipse_bound, color=(255, 0, 0), thickness=1)

                        up = np.array([0, -1])
       
                        if len(target.history) > 50: 
                            refresh = output_callback(np.dot(history_center - ellipse_center, up), radius)
                            if refresh:
                                target.history.clear()

                cv2.imshow(self.window_title, frame)

                key = cv2.waitKey(20)
                escape = 27
                if key == ord("q") or key == escape:
                    break

        except KeyboardInterrupt:
            return
    
        finally:
            self._release()


if __name__ == "__main__":
    tracker = ObjectTracking()
    tracker.run()

