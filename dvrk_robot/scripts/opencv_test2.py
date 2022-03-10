#!/usr/bin/env python

from turtle import back
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

def process(frame, reference):
    # Do we blur? and if so when?
    delta = cv2.absdiff(frame, reference)
    motion = cv2.threshold(delta, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel_size = 10
    morph_element = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (2 * kernel_size + 1, 2 * kernel_size + 1),
        (kernel_size, kernel_size)
    );
    roi = cv2.dilate(motion, morph_element);
    return roi

background_frames_count = 0
background_frames = []
frames_per_reference = 10

def collect_reference(frame):
    global background_frames
    global background_frames_count

    if background_frames_count % frames_per_reference == 0:
        background_frames.append(frame)

    background_frames_count += 1


window_title = "OpenCV calibration test #1"

create_window(window_title)
init_video()

try:
    while retval and len(background_frames) < 100:
        retval, frame = video_capture.read()
        collect_reference(frame)

    background_reference = cv2.mean(background_frames)

    cv2.imshow(window_title, background_reference)

    key = cv2.waitKey(20)
    if key == 27:
        exit()

    while retval:
        retval, frame = video_capture.read()
        roi = process(frame, background_reference)
        cv2.imshow(window_title, roi)

        key = cv2.waitKey(20)
        if key == 27:
            break

except KeyboardInterrupt:
    pass

release()
