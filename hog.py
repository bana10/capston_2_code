import numpy as np
import cv2
import time
import RPi.GPIO as GPIO

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

x_servo_pin = 18 
y_servo_pin = 19
GPIO.setmode(GPIO.BCM)
GPIO.setup(x_servo_pin, GPIO.OUT)
GPIO.setmode(GPIO.BCM)
GPIO.setup(y_servo_pin, GPIO.OUT)

x_pwm = GPIO.PWM(x_servo_pin, 50) 
y_pwm = GPIO.PWM(y_servo_pin, 50) 

x_pwm.start(0)
y_pwm.start(0)

cap = cv2.VideoCapture(0)
prevt = time.time()

def X_Motor(angle):
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5 
    x_pwm.ChangeDutyCycle(duty_cycle)

def Y_Motor(angle):
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5 
    y_pwm.ChangeDutyCycle(duty_cycle)


wp = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
hp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

Xspin = 45
Yspin = 20

goleft = True

X_Motor(0)
Y_Motor(0)

while(True):
    ret, frame = cap.read()
    #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    boxes, weights = hog.detectMultiScale(frame, winStride=(8,8))
    maxx = 0
    minx = wp
    maxy = 0
    miny = hp

    if len(boxes) > 0:
        for (x,y,w,h) in boxes:
            cv2.rectangle(frame, (x,y), (x + w, y + h), (50,200,50), 2)
            center_x = x + int(w / 2)
            center_y = y + int(h / 2)

            if center_x > maxx:
                maxx = center_x
            if center_x < minx:
                minx = center_x
            if center_y > maxy:
                maxy = center_y
            if center_y < miny:
                miny = center_y

        minanglex = int((minx / wp * (Xspin * 2)) - Xspin)
        maxanglex = int((maxx / wp * (Xspin * 2)) - Xspin)

        minangley = int((miny / hp * (Yspin * 2)) - Yspin)
        maxangley = int((maxy / hp * (Yspin * 2)) - Yspin)

        if goleft:
            X_Motor(minanglex)
        else:
            X_Motor(maxanglex)
        
        Y_Motor(int((minangley + maxangley) / 2))
        goleft = not goleft
    
    cv2.imshow('asdf',frame)
    if cv2.waitKey(1) == ord('q'):
        break

    print(time.time() - prevt)
    prevt = time.time()
    print(len(boxes))