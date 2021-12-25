# Master code for sugar launching robot.
# Made by Equals Engineering.
# To see the robot in action visit: https://youtu.be/oIwLhZZftGw
# 
# This code serves as notes for its author and for curious people that wants to see it.
# There is room for optimization and some parts are still work in progress.
# Under no circumstances is this code to be used for projectile devices to cause harm.
#
# Run "sudo pigpiod" in the pi terminal before running the code for the first time
# to set up pigpio library and avoid errors.

#Imports for servo and math
from gpiozero import Servo
import math
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory
#For ulrasound sensor
import RPi.GPIO as GPIO
import time
#OpenCV for camera
import cv2
import numpy as np

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT) # Right motor backwards
GPIO.setup(27, GPIO.OUT) # Right motor forward
GPIO.setup(22, GPIO.OUT) # Left motor backwards
GPIO.setup(23, GPIO.OUT) # Left motor forward

#Ultrasound sensor
TRIG=21
ECHO=20

#Camera stuff
cap = cv2.VideoCapture(0)
cap.set(3, 480)
cap.set(4, 320)
_, frame = cap.read()
rows, cols, _ = frame.shape  
center = int(cols / 2) #center pixel in x
loopValue = True

#Servo and equations
factory = PiGPIOFactory()
bigServo = Servo(25, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
smallServo = Servo(24, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, pin_factory=factory)
bigServoStartPos = 1
smallServoStartPos = 0
smallServoHookPos = 1
bigServo.value = bigServoStartPos
smallServo.value = smallServoStartPos

# Find the spring displacement x Given the desired launch distance d.
k = 880                        # Spring constant (Approximated)
x_max = 22*pow(10, -3)         # Maximum spring displacement
alpha = math.pi/4              # Launch angle in radians (is 45 degrees)
ms = 3.6*pow(10, -3)           # Mass of suggar cube, 20 cubes =72g => 3.6g
ml = 4*pow(10, -3)             # Estimated mass of launcher and fishingline.
mt = ms + ml                   # Combined mass of ms and ml 
h = 0.14                       # Hight of suggar cube at launch
g = 9.81                       # Gravity constant
#d = 0.50                      # Desired manual distance [m] for the suggar cube to be launched 
wheel_ratio = 26.8*pow(10, -3) 
# The spring displacement needs an offset since the target is inside the cup and not
# infront of it. The offset also helps to calibrate the decreased initial velocity due to
# friction.
# 6.8 for red cup, 7.45 for red paper on black and white cup (larger offset gives more force)
offset = 6.8*pow(10, -3)     

#-------------Functions--------------
#-----Measure distance-----
#Use the ultrasound sensor to measure the distance
def getDistance():
    for t in range(0, 10):
        #print("distance measurement in progress")
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)
        GPIO.output(TRIG,False)
        #print("waiting for sensor to settle")
        time.sleep(0.2)
        GPIO.output(TRIG,True)
        time.sleep(0.00001)
        GPIO.output(TRIG,False)
        while GPIO.input(ECHO)==0:
            pulse_start=time.time()
        while GPIO.input(ECHO)==1:
            pulse_end=time.time()
        pulse_duration=pulse_end-pulse_start
        distance=pulse_duration*17150
        distance=round(distance,2)*pow(10,-2) # [m]
        time.sleep(0.00001)
        print("distance:",distance,"m")
        #time.sleep(1)
    return distance

#------------------------------
def getSpringDisplacement(d):
    #The math is derived from Hook's law, Newtons second law and kinematics.
    #Initial velocity (vi)
    vi = math.sqrt(((-g)*pow(d, 2))/(2*(-h-d)*pow(math.cos(alpha), 2)))
    #Spring displacement (x)
    x = (2*g*mt*math.cos(alpha)+math.sqrt(4*pow(g, 2)*pow(mt, 2)*pow(math.cos(alpha), 2)+8*k*mt*pow(vi, 2)))/(4*k)
    x_with_offset = x + offset
    #Convert spring displacement to servo rotation
    bigServoLaunchValue = 1 - (x_with_offset / wheel_ratio)
    #Printouts and data for equation and servos
    #print("x_with_offset is:",x_with_offset)
    #print("bigServoLaunchValue is:",bigServoLaunchValue)
    #print("x is:",x)
    #print("vi is:",vi)
    return bigServoLaunchValue

#-----Get the robot positioned so the cup is right infront of it-----
def getPositioned():
    running = True
    low_red = np.array([161,155,84])
    high_red = np.array([179,255,255])
    rightMotorTurns = 0
    leftMotorTurns = 0
    while running:
        for j in range(0, 10):
            exitloop = False
            _, frame = cap.read()
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
            #Red color
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            _, contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
            x_medium = int(cols / 2) #Set initial Average x position of red object to the middle
    
            for cnt in contours:
                (x, y, w, h) = cv2.boundingRect(cnt)
        
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                x_medium = int((x + x + w) / 2)
                break
    
            cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)
            #cv2.imshow("", red_mask)
            cv2.imshow("Frame", frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        
            #move wheels with motor
        if x_medium < center - 7:
            startRightMotorForward()
            sleep(0.1)
            stopRightMotorForward()
            rightMotorTurns += 1
            #sleep(0.05)
        elif x_medium > center + 7:
            startLeftMotorForward()
            sleep(0.1)
            stopLeftMotorForward()
            leftMotorTurns += 1
        if x_medium > center - 7 and x_medium < center + 7:
            running = False
            return rightMotorTurns, leftMotorTurns

#-----DC motor control functions-----
def startRightMotorForward():
    GPIO.output(27, 1)
def stopRightMotorForward():
    GPIO.output(27, 0)
def startRightMotorBackward():
    GPIO.output(17, 1)
def stopRightMotorBackward():
    GPIO.output(17, 0)
def startLeftMotorForward():
    GPIO.output(23, 1)
def stopLeftMotorForward():
    GPIO.output(23, 0)
def startLeftMotorBackward():
    GPIO.output(22, 1)
def stopLeftMotorBackward():
    GPIO.output(22, 0)
    
# Go home to original spot (Work in progress)
def getBack(rightMotorTurns, leftMotorTurns):
    
    startRightMotorBackward()
    sleep(0.1*rightMotorTurns)
    stopRightMotorBackward()
    
    startLeftMotorBackward()
    sleep(0.1*leftMotorTurns)
    stopLeftMotorBackward()
    
def celebrationBackForward():
    
    startRightMotorBackward()
    startLeftMotorBackward()
    sleep(0.3)
    stopRightMotorBackward()
    stopLeftMotorBackward()
    sleep(0.001)
    startRightMotorForward()
    startLeftMotorForward()
    sleep(0.3)
    stopRightMotorForward()
    stopLeftMotorForward()
    
def celebrationCircleLmBack():
    
    startLeftMotorBackward()
    sleep(1.9)
    stopLeftMotorBackward()
            
while True:
    rightMotorTurns, leftMotorTurns = getPositioned()
    d = getDistance()
    bigServoLaunchValue = getSpringDisplacement(d)  
    # Depending on the servo the values between -1 and 1 corresponds to 0 - 180 degrees
    # or 0 to 270 degrees.
    #Small servo
    sleep(1)
    smallServo.value = smallServoHookPos
    sleep(1)
    bigServo.value = bigServoLaunchValue # bigServoLaunchValue between -1 and 1.
    #Start pos for big servo is
    sleep(2)
    smallServo.value = smallServoStartPos
    sleep(0.3)
    #celebrationCircleLmBack()
    #celebrationBackForward()
    sleep(7)
    bigServo.value = bigServoStartPos
    sleep(1)
    #getBack(rightMotorTurns, leftMotorTurns)




























