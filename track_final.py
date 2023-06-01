import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import board
import Adafruit_DHT
from ctypes import cdll
import joblib
import sklearn

prev_x = 0
prev_y = 0

# 서보모터 gpio
MOTOR_X = 18
MOTOR_Y = 19

# 선풍기 모터
MOTOR_PIN = 22

# PWM 주파수 설정
PWM_FREQUENCY = 50

# PWM 사이클 범위
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

# gpio 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_X, GPIO.OUT)
GPIO.setup(MOTOR_Y, GPIO.OUT)
#GPIO.setup(MOTOR_PIN, GPIO.OUT)



pwm_x = GPIO.PWM(MOTOR_X, PWM_FREQUENCY)
pwm_y = GPIO.PWM(MOTOR_Y, PWM_FREQUENCY)
#pwm = GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)
pwm_x.start(0)
pwm_y.start(0)
#pwm.start(0)

x_model = joblib.load('./Xloc_model.pkl')
y_model = joblib.load('./Yloc_model.pkl')


def move_motors(x, y):
    # 각 서보 모터 위치에 대한 PWM 듀티 사이클 계산
    duty_cycle_x = ((x / 640) * (DUTY_CYCLE_MAX -
                    DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    duty_cycle_y = ((y / 480) * (DUTY_CYCLE_MAX -
                    DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN

    # 서보 모터를 지정된 위치로 이동
    pwm_x.ChangeDutyCycle(duty_cycle_x)
    pwm_y.ChangeDutyCycle(duty_cycle_y)


class Temperature(object):
    def __init__(self, libPath):
        self.lib = cdll.LoadLibrary(libPath)
        self.obj = self.lib.Temperature_new()

    def check(self, tick=0.5):
        self.lib.Temperature_check(self.obj)
        time.sleep(tick)
        

def measure_temperature(obj_class):
    if obj_class in ["person"]:
        f = Temperature(libPath='./temperature.so')
        result = f.check()
        object_temperature = result.split(", ")[1]
        return object_temperature

def detect_and_measure_temperature():
    # Camera operation
    cap = cv2.VideoCapture(0)
    net = cv2.dnn.readNet("yolov2-tiny.weights", "yolov2-tiny.cfg")

    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = net.getUnconnectedOutLayersNames()
    print("Camera initialized")
    while True:
        ret, frame = cap.read()

        # Object detection
        height, width, channels = frame.shape
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)
    
        objects = []
    
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                class_name = classes[class_id]
    
                if confidence > 0.25 and class_name in ["person"]:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
    
                    obj = {
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "x": x,
                        "y": y,
                        "w": w,
                        "h": h
                    }
    
                    objects.append(obj)
        print("Objects detected")

        # Extract center points
        center_x = 0
        center_y = 0
        num_objects = len(objects)
        global prev_x
        global prev_y

        # 중심점을 시리얼 통신으로 보내기
        temperature = -1
        humidity = -1
        print("detected:", num_objects)
        # 상황에 따라 온도모듈 선택해서 온도 특정
        if num_objects == 1:
            cv2.rectangle(frame, (obj['x'], obj['y']), (obj['x'] + obj['w'], obj['y'] + obj['h']), (255,0,0), 2)
            obj = objects[0]
            temperature = measure_temperature(objects)
            center_x = obj["x"] + obj['w'] / 2
            center_y += obj["y"] + obj["h"] / 2
            p_x = center_x
            p_y = center_y
            
            if (prev_y == prev_x and prev_x == 0) is False:
                dat = [[prev_x, prev_y, center_x, center_y]]
                p_x = int(x_model.predict(dat))
                p_y = int(y_model.predict(dat))

            move_motors(p_x, p_y)
            prev_x = center_x
            prev_y = center_y
       
        elif num_objects >= 2:
            for obj in objects:
                center_x += obj["x"] + obj["w"] / 2
                center_y += obj["y"] + obj["h"] / 2
            if num_objects > 0:
                center_x /= num_objects
                center_y /= num_objects
            # DHT11 
            #humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        
        cv2.imshow("Image", frame)
        time.sleep(0.25)
    
    cap.release()


if __name__ == "__main__":
    detect_and_measure_temperature()
    GPIO.cleanup()




