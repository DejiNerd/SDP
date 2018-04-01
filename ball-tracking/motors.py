import RPi.GPIO as GPIO
import time
##18 = white, 23 = green, 24 = blue
#in relation to the H-bridge, side with inputs = forward
#connect + of motor to side with inputs for both motors
#we're using 3.3v DC power for en 1
def init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(27, GPIO.OUT)
    GPIO.setup(26, GPIO.OUT)  
    GPIO.setup(20, GPIO.OUT) 
    
def forward(tf):
    #counter clockwise left, clockwise right
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.HIGH)
    time.sleep(tf)
def stop(tf):
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    time.sleep(tf)
    #GPIO.cleanup()
def right(tf):
    #counter clockwise left, stop right
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.HIGH)
    time.sleep(tf)
    #GPIO.cleanup()
def left(tf):
    #clockwise right, stop left 
    GPIO.output(18, GPIO.LOW)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)
    time.sleep(tf)
    #GPIO.cleanup()
init()
p = GPIO.PWM(26, 100)
g = GPIO.PWM(20, 100)
p.start(100)
g.start(100)
forward(1)
p.ChangeDutyCycle(50.5)
g.ChangeDutyCycle(50.5)
forward(1)
p.ChangeDutyCycle(25)
g.ChangeDutyCycle(25)
forward(1)
GPIO.cleanup()
