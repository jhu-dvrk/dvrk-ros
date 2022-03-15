#!/usr/bin/env python

import cv2
import numpy as np
import collections

display_output = True
video_capture = None
retval = None

def create_window(window_title, mouse_callback):
    cv2.namedWindow(window_title)
    cv2.setMouseCallback(window_title, mouse_callback)

def init_video():
    global video_capture
    global retval
    video_capture = cv2.VideoCapture(0)
    if video_capture.isOpened():
        retval, _ = video_capture.read()
    else:
        print("Couldn't read from camera.")
        retval = False

def release():
    global video_capture
    video_capture.release()
    cv2.destroyAllWindows()


class TrackedObject:
    def __init__(self, detection):
        self.position = (detection[0], detection[1])
        self.size = detection[2]
        self.strength = 1
        self.history = collections.deque(maxlen=400)
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


class Tracking:
    def __init__(self, min_distance):
        self.objects = []
        self.minimum = min_distance**2
        self.primaryTarget = None

    def updatePrimary(self, position):
        nearbyObjects = [x for x in self.objects if x.distanceTo(position) < self.minimum]
        self.primaryTarget = nearbyObjects[0] if len(nearbyObjects) == 1 else None

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


def process(frame, tracker):
    blurred = cv2.medianBlur(frame, 5)
    inverted = ~blurred
    cv2.imshow("inverted", inverted)
    #cv2.imwrite("./inverted.png", inverted)
    hsv = cv2.cvtColor(inverted, cv2.COLOR_BGR2HSV)
    thresholded = cv2.inRange(hsv, (75, int(25*2.55), int(50*2.55)), (105, int(75*2.55), int(100*2.55)))
    
    kernel_size = 3
    morph_element = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (2 * kernel_size + 1, 2 * kernel_size + 1),
        (kernel_size, kernel_size)
    )
    image = cv2.erode(thresholded, morph_element)
    image = cv2.dilate(image, morph_element)
    image = cv2.dilate(image, morph_element)

    contours, hierarchies = cv2.findContours(
        image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    moments = [cv2.moments(c) for c in contours]
    detections = [(int(M['m10']/M['m00']), int(M['m01']/M['m00']), 5) for M in moments if M['m00'] != 0]

    tracker.register(detections)


window_title = "OpenCV calibration test"
tracker = Tracking(60)

def on_click(event, x, y, flags, params):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    tracker.updatePrimary((x, y))

create_window(window_title, on_click)
init_video()

try:
    while retval:
        retval, frame = video_capture.read()
        process(frame, tracker)

        target = tracker.primaryTarget
        if target is not None and target.strength >= 12:
            cv2.circle(frame, target.position, radius=3, color=(0, 0, 255), thickness=cv2.FILLED)
            ellipse_bound = cv2.fitEllipse(np.array(target.history))
            cv2.ellipse(frame, ellipse_bound, color=(255, 0, 0), thickness=1)

        cv2.imshow(window_title, frame)

        key = cv2.waitKey(20)
        if key == 27:
            break

except KeyboardInterrupt:
    pass

release()

