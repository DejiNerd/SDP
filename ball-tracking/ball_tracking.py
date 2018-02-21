# Created by BallBot SDP, using the Open Source Computer Vision Library
# USAGE: python ball_tracking.py
# (source ~./profile; workon cv; python ~/SDP/code/ball-tracking.py)&

from collections import deque
import cv2
import RPi.GPIO as GPIO
import numpy as np
import imutils
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())


def setup():
    # LED setup
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(2, GPIO.OUT)  # Bryce motors


def goForward():
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.setup(2, GPIO.LOW)


def goLeft():
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.setup(2, GPIO.HIGH)


def goRight():
    GPIO.output(18, GPIO.HIGH)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.setup(2, GPIO.HIGH)


def moveForwardABit():
    GPIO.output(18, GPIO.HIGH)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)


def roomba():
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)


def shutdown():
    # cleanup the camera and close any open windows
    camera.release()
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.setup(2, GPIO.LOW)
    cv2.destroyAllWindows()


setup()

# define the lower and upper boundaries of the tennis ball color
# ball in the HSV color space, then initialize the list of tracked points
lowerColorBound = (29, 86, 6)
upperColorBound = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

# grab the reference to the webcam
camera = cv2.VideoCapture(0)  # Capture Video...
print("Camera warming up ...")

GPIO.setup(2, GPIO.LOW)
ballCount = 0
while True:
    # grab the current frame
    (captured, frame) = camera.read()

    # if we are viewing a video and we did not capture a frame,
    # then we have reached the end of the video
    if args.get("video") and not captured:
        break

    # resize the frame, and convert it to the HSV color space
    width = 200
    frame = imutils.resize(frame, width)
    cv2.imshow("before", frame)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV", hsv)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, lowerColorBound, upperColorBound)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # cv2.imshow("mask", mask)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        leftBound = (width / 2) - 30
        rightBound = (width / 2) + 30

        if (center[0] >= leftBound and center[0] <= rightBound):
            goForward()

        elif (center[0] < leftBound):
            goRight()

        else:
            goLeft()

        # print('center: ', center, 'radius', int(radius))  # outputs coordinate to command line
        # cv2.circle(image, center, radius, color, thickness)
        cv2.circle(frame, center, 5, (255, 0, 0), -1)

        # draw outer circle  if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            # cv2.circle(image, center, radius, color, thickness)
            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 0, 255), 2)

        if radius >= 50:
            # ball is close enough to be retrieved!
            ballCount = ballCount + 1  # look into this!
            print('Ball Retrieved ' + str(ballCount))
            moveForwardABit()
    else:
        roomba()

    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in xrange(1, len(pts)):
        # if either of the tracked points are None, ignore them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line anddraw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        # cv.Line(img, pt1, pt2, color, thickness=1, lineType=8, shift=0)
        cv2.line(frame, pts[i - 1], pts[i], (255, 0, 0), thickness)

    # show the frame to our screen
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

shutdown()
