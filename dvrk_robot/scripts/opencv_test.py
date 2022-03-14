#!/usr/bin/env python

import cv2
import numpy as np

display_output = True
video_capture = None
retval = None

def create_window(window_title):
    cv2.namedWindow(window_title)

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

    def distanceTo(self, position):
        dx = self.position[0] - position[0]
        dy = self.position[1] - position[1]
        return dx*dx + dy*dy

    def match(self, detection):
        self.position = (detection[0], detection[1])
        self.size = detection[2]
        self.strength = min(self.strength+1, 20)

    def noMatch(self):
        self.strength -= 2


class Tracking:
    def __init__(self, min_distance):
        self.objects = []
        self.minimum = min_distance**2

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
        else:
            for d in detections:
                self.objects.append(TrackedObject(d)) 

tracker = Tracking(30)

def process(frame):
    inverted = ~frame
    #cv2.imwrite("./inverted.png", inverted)
    hsv = cv2.cvtColor(inverted, cv2.COLOR_BGR2HSV)
    thresholded = cv2.inRange(hsv, (75, int(15*2.55), int(40*2.55)), (105, int(75*2.55), int(100*2.55)))
    
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
        image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    moments = [cv2.moments(c) for c in contours]
    detections = [(int(M['m10']/M['m00']), int(M['m01']/M['m00']), 5) for M in moments if M['m00'] != 0]

    tracker.register(detections)

window_title = "OpenCV calibration test"

create_window(window_title)
init_video()

try:
    while retval:
        retval, frame = video_capture.read()
        process(frame)

        print(len([x for x in tracker.objects if x.strength >= 12]))

        for obj in tracker.objects:
            if obj.strength < 12:
                continue

            (x, y) = obj.position
            r = obj.size
            # draw the outer circle
            cv2.circle(frame, (x, y),r,(0,255,0),2)
            # draw the center of the circle
            cv2.circle(frame, (x, y),2,(0,0,255),3)

        cv2.imshow(window_title, frame)

        key = cv2.waitKey(20)
        if key == 27:
            break

except KeyboardInterrupt:
    pass

release()

