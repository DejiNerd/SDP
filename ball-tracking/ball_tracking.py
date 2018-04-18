# Created by BallBot SDP, using the Open Source Computer Vision Library
# USAGE: python ball_tracking.py
import random
import argparse
import time
from collections import deque
import RPi.GPIO as GPIO
import cv2
import imutils
import serial
import numpy as np
import BluetoothTest as bluetooth

ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=64,
                help="max buffer size")
args = vars(ap.parse_args())
pts = deque(maxlen=args["buffer"])
LED_PAUSE = 17
LED_ACTIVE = 22
PAUSE_SWITCH = 5
moveForward, ntime, ballCount, switch, p, g, gcount, rooomba = 0, 0, 0, 0, 0, 0, 0, 0
state = None
lowerColorBound = (29, 86, 6)
upperColorBound = (64, 255, 255)


def setup():
    global p, g
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(PAUSE_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LED_PAUSE, GPIO.OUT)
    GPIO.setup(LED_ACTIVE, GPIO.OUT)
    ########### Bryce motors ###############
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(27, GPIO.OUT)  
    ########   PWM  #######################
    GPIO.setup(20, GPIO.OUT)  # en b = left motor
    GPIO.setup(26, GPIO.OUT)  # en a = right motor
    g = GPIO.PWM(20, 100)
    p = GPIO.PWM(26, 100)
    p.start(100)
    g.start(100)
    stop()


def goForward():
    p.ChangeDutyCycle(100)
    g.ChangeDutyCycle(100)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.HIGH)


def goLeft():
    #if dist > width/4:
    # g.ChangeDutyCycle(0)
    #else:
    #g.ChangeDutyCycle(90)
    p.ChangeDutyCycle(100)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)


def goRight():
    #if dist > width/4:
    #p.ChangeDutyCycle(0)
    #else:
    #p.ChangeDutyCycle(90)
    g.ChangeDutyCycle(100)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.HIGH)


# TODO
def moveForwardABit():
    global ntime, moveForward
    ntime = time.time()
    moveForward = True
    print("moveforwardabt", ntime)


# TODO
def roomba():
    global ntime, rooomba, noball
    ntime = time.time()
    rooomba = True
    noball = True

def turnInPlace():
    p.ChangeDutyCycle(100)
    g.ChangeDutyCycle(100)
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(27, GPIO.LOW)

def stop():
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)


def pause():
    stop()
    GPIO.output(LED_PAUSE, GPIO.HIGH)
    GPIO.output(LED_ACTIVE, GPIO.LOW)


def resume():
    GPIO.output(LED_PAUSE, GPIO.LOW)
    GPIO.output(LED_ACTIVE, GPIO.HIGH)


def shutdown():
    ser.close()
    camera.release()
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    GPIO.output(LED_PAUSE, GPIO.LOW)
    GPIO.output(LED_ACTIVE, GPIO.LOW)
    cv2.destroyAllWindows()


setup()
camera = cv2.VideoCapture(0)  # Capture Video from web cam...
sw ='0'
ser = serial.Serial('/dev/rfcomm0',9600)
time.sleep(2)
print("Camera warming up ...")

while True:
##    try:
##        ch0 = ser.read()
##        sw = str(ch0.decode("utf-8")).strip(' \t\n\r')
##    except serial.SerialException:
##        print 'OUZT~~'
##        sw = '0'
    sw = bluetooth.bt(ser,sw)
    if sw ==  '0':
        pause()
    elif sw ==  '1':
        resume()
        (captured, frame) = camera.read()
        if args.get("video") and not captured:
            break
        
        # resize the frame, and convert it to the HSV color space
        width = 200
        frame = imutils.resize(frame, width)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # construct a mask for the color "yellowish/green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, lowerColorBound, upperColorBound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        ## cv2.imshow("blurredmask", mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if rooomba:
            if (gcount % 4) == 0:
                goRight()
                print ("360")
                if time.time() > ntime + 8: 
                    stop()
                    rooomba= False
                    gcount = gcount + 1
                    # time.sleep(2)
            else:
                if state is None:
                    randint = random.randint(1,3)
                    if randint % 3 == 0:
                        state = 'right'
                    elif randint % 3 == 1:
                        state = 'left'
                    elif randint % 3 == 2:
                        state = 'forward'
                elif state == 'right':
                        goRight()
                        print ("R")
                        if time.time() > ntime + 3: 
                            stop()
                            state = None
                            rooomba= False
                            gcount = gcount + 1
                            # time.sleep(2)
                elif state == 'left':
                    goLeft()
                    print("L")
                    if time.time() > ntime + 3: 
                        stop()
                        state = None
                        rooomba= False
                        gcount = gcount + 1
                        # time.sleep(2)
                elif state == 'forward':
                    goForward()
                    print ("F")
                    if time.time() > ntime + 2: 
                        stop()
                        state = None
                        rooomba= False
                        gcount = gcount + 1
                        # time.sleep(2)
        if moveForward:
            goForward()
            if time.time() > ntime + .25:
                stop()
                moveForward = False
                ballCount = ballCount + 1
                print('Ball Retrieved ' + str(ballCount))
        # only proceed if at least one contour was found
        elif len(cnts) > 0:
            gcount = 0
            rooomba = False
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(frame, center, 5, (255, 0, 0), -1)
            print('center: ', center, 'radius', int(radius))
            leftBound = (width / 2) - (0.15 * width)
            rightBound = (width / 2) + (0.15 * width)

            if leftBound <= center[0] <= rightBound:
                goForward()

            elif center[0] < leftBound:
                goLeft()
            else:
                goRight()
            # draw outer circle if the radius meets a minimum size
            if radius > 0.05 * width:
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 0, 255), 2)

            if .15 * width >= radius >= 0.1325 * width:
                # ball is close enough to be retrieved!
                moveForwardABit()
        elif not rooomba:
            goForward()
            time.sleep(0.5)
            roomba()
        
        pts.appendleft(center)  # update the points queue

        # loop over the set of tracked points
##        for i in xrange(1, len(pts)):
##            if pts[i - 1] is None or pts[i] is None:
##                continue
##            # otherwise, compute the thickness of the line and draw the connecting lines
##            thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
##            cv2.line(frame, pts[i - 1], pts[i], (255, 0, 0), thickness)

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break


shutdown()
