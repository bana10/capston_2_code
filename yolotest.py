import numpy as np
import cv2
import time
import RPi.GPIO as GPIO

import joblib
import sklearn

x_model = joblib.load('./Xloc_model.pkl')
y_model = joblib.load('./Yloc_model.pkl')


cap = cv2.VideoCapture(0)
prevt = time.time()
x_servo_pin = 13
y_servo_pin = 19

prev_x = 0
prev_y = 0
net = cv2.dnn.readNet("yolov2-tiny.weights", "yolov2-tiny.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayersNames()

wp = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
hp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

Xspin = 45
Yspin = 20

goleft = True

try:
    while(True):
        ret, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    # YOLOv3-tiny 모델로 객체 감지
        net.setInput(blob)
        boxes = net.forward(output_layers)
        maxx = 0
        minx = wp
        maxy = 0
        miny = hp

        if len(boxes) > 0:
            goleft = not goleft
        else:
            prev_x = 0
            prev_y = 0

        # cv2.imshow('asdf', frame)
        # if cv2.waitKey(1) == ord('q'):
        #     X_Motor(0)
        #     GPIO.cleanup()
        #     break

        print(time.time() - prevt)
        prevt = time.time()
        print(len(boxes))

except KeyboardInterrupt:
    GPIO.cleanup()
