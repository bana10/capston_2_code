import numpy as np
import cv2
import time
import RPi.GPIO as GPIO


cap = cv2.VideoCapture(0)
prevt = time.time()

x_servo_pin = 13
y_servo_pin = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(x_servo_pin, GPIO.OUT)
GPIO.setup(y_servo_pin, GPIO.OUT)

x_pwm = GPIO.PWM(x_servo_pin, 50)
y_pwm = GPIO.PWM(y_servo_pin, 50)

x_pwm.start(0)
y_pwm.start(0)

prev_x = 0
prev_y = 0

net = cv2.dnn.readNet("yolov2-tiny.weights", "yolov2-tiny.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayersNames()

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
Y_Motor(90)

try:
    while(True):
        ret, frame = cap.read()
        #gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    # YOLOv3-tiny 모델로 객체 감지
        net.setInput(blob)
        outs = net.forward(output_layers)
        maxx = 0
        minx = wp
        maxy = 0
        miny = hp
        class_ids = []
        confidences = []
        boxes = []
        centers = []
        height, width, channels = frame.shape
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # 객체가 발견되었을 때
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # 객체 바운딩 박스 좌표 계산
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    centers.append((center_x, center_y))
        
        boxes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        if len(boxes) > 0:
            inhuman = 0
            for i in boxes.flatten():
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                
                if label in ['person']:
                    inhuman += 1
                    cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (50, 200, 50), 2)
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
            if inhuman != 0:
                minanglex = int((minx / wp * (Xspin * 2)) - Xspin)
                maxanglex = int((maxx / wp * (Xspin * 2)) - Xspin)

                minangley = int((miny / hp * (Yspin * 2)) - Yspin)
                maxangley = int((maxy / hp * (Yspin * 2)) - Yspin)

                if goleft:
                    X_Motor(-1 * minanglex)
                    print("goleft:",  -1 * minanglex)
                else:
                    X_Motor(-1 * maxanglex)
                    print("goright:",  -1 * maxanglex)

                yangleavg = int((minangley + maxangley) / 2)
                if yangleavg > 60:
                    Y_Motor(yangleavg)
                else:
                    Y_Motor(60)
                print("avgy:", yangleavg)
                goleft = not goleft
        else:
            prev_x = 0
            prev_y = 0

        cv2.imshow('asdf', frame)
        if cv2.waitKey(1) == ord('q'):
            X_Motor(0)
            GPIO.cleanup()
            break

        print(time.time() - prevt)
        prevt = time.time()
        print(len(boxes))

except KeyboardInterrupt:
    X_Motor(0)
    GPIO.cleanup()
