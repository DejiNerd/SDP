#!/bin/bash

source /home/pi/.profile
workon cv
cd /home/pi/SDP/ball-tracking/
python ball_tracking.py > output
