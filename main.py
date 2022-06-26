#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, Joseph Dream
* @file         FaceTracking.py
* @version      V2.0
* @details
* @par History

@author: josephdream
"""
import numpy as np
import cv2
import os
import time  
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import threading

import sys
reload(sys)
sys.setdefaultencoding('utf8')

#Initialize PCA9685 and servo
servo_pwm = Adafruit_PCA9685.PCA9685()  # Example if the servo gimbal

# Set the initial value of the servo, you can debug it according to your own requirements
servo_pwm.set_pwm_freq(60)  # Set the frequency to 60HZ
servo_pwm.set_pwm(5,0,325)  # base servo
servo_pwm.set_pwm(4,0,325)  # Tilt servo
time.sleep(1)

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

#Initialize the camera and set the threshold
usb_cap = cv2.VideoCapture(0)
# Set the display resolution, set to 320 × 240 px
usb_cap.set(3, 320)
usb_cap.set(4, 240)

pid_x=0
pid_y=0
pid_w=0
pid_h=0

#Each degree of freedom of the servo gimbal requires 4 variables
pid_thisError_x=0   
pid_lastError_x=0   
pid_thisError_y=0
pid_lastError_y=0

# The rotation angle of the servo
pid_X_P = 330
pid_Y_P = 330   

pid_flag=0
makerobo_facebool = False

# initialize LED GPIO
redLed = 21    # LED
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(redLed, GPIO.OUT)


# Robot servo rotation
def Robot_servo():
    while True:
        servo_pwm.set_pwm(5,0,650-pid_X_P)
        servo_pwm.set_pwm(4,0,650-pid_Y_P)


servo_tid=threading.Thread(target=Robot_servo)  
servo_tid.setDaemon(True)
servo_tid.start()                               

# Start with LED off
GPIO.output(redLed, GPIO.LOW)
ledOn = False

while 1:
    ret,frame = usb_cap.read()       
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x,y,w,h) in faces:
        #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = frame[y:y+h, x:x+w]

        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

    max_face=0
    value_x=0
    # find the face and draw the rectangle
    if len(faces)>0:
        (pid_x,pid_y,pid_w,pid_h) = faces[0]
        cv2.rectangle(frame,(pid_x,pid_y),(pid_x+pid_h,pid_y+pid_w),(0,255,0),2)
        result=(pid_x,pid_y,pid_w,pid_h)
        pid_x=result[0]+pid_w/2
        pid_y=result[1]+pid_h/2
        makerobo_facebool = True      

        # Error value processing
        pid_thisError_x=pid_x-160
        pid_thisError_y=pid_y-120

        #Adjust the two values ​​of P and D by yourself, and detect the influence of the changes of the two values ​​on the stability of the steering gear
        pwm_x = pid_thisError_x*5+1*(pid_thisError_x-pid_lastError_x)
        pwm_y = pid_thisError_y*5+1*(pid_thisError_y-pid_lastError_y)
        
        #Iterative error value operation
        pid_lastError_x = pid_thisError_x
        pid_lastError_y = pid_thisError_y
        
        pid_XP=pwm_x/100
        pid_YP=pwm_y/100
        
        # pid_X_P pid_Y_P is the final PID value
        pid_X_P=pid_X_P+int(pid_XP)
        pid_Y_P=pid_Y_P+int(pid_YP)
        
        # Light up the LED light
        GPIO.output(redLed, GPIO.HIGH)

        #The limit servo is within a certain range
        if pid_X_P>650:
            pid_X_P=650
        if pid_X_P<0:
            pid_X_P=0
        if pid_Y_P>650:
            pid_Y_P=650
        if pid_X_P<0:
            pid_Y_p=0

    
    else:
        GPIO.output(redLed, GPIO.LOW)
        
    
    cv2.imshow("MAKEROBO Robot", frame)
    if cv2.waitKey(1)==119:
        break
# do a bit of cleanup
print("\n [INFO] Exiting Program and cleanup stuff \n")
GPIO.cleanup()
cap.release()
cv2.destroyAllWindows()
