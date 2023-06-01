import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import board
import busio
import Adafruit_DHT
from ctypes import cdll


DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(DHT_PIN, GPIO.IN)



class Temperature(object):
    def __init__(self, libPath):
        self.lib = cdll.LoadLibrary(libPath)
        self.obj = self.lib.Temperature_new()

    def check(self, tick=0.5):
        self.lib.Temperature_check(self.obj)
        time.sleep(tick)
        
def detect_objects(frame):
    # YOLO configuration
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    objects = []
    
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            class_name = classes[class_id]
            
            if confidence > 0.5 and class_name in ["person", "cat", "dog"]:
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
    
    return objects

def measure_temperature(obj_class):
    if obj_class in ["person", "cat", "dog"]:
        f = Temperature(libPath='./temperature.so')
        result = f.check()
        object_temperature = result.split(", ")[1]
        return object_temperature

def detect_and_measure_temperature():
    # Camera operation
    cap = cv2.VideoCapture(0)
    print("Camera initialized")
    ret, frame = cap.read()
    cap.release()

    # Object detection
    objects = detect_objects(frame)
    print("Objects detected")

    # Extract center points
    center_x = 0
    center_y = 0
    num_objects = len(objects)
    for obj in objects:
        center_x += obj["x"] + obj["w"] / 2
        center_y += obj["y"] + obj["h"] / 2
    if num_objects > 0:
        center_x /= num_objects
        center_y /= num_objects

    # 중심점을 시리얼 통신으로 보내기
    serial_port = serial.Serial('/dev/ttyS0', 115200)
    serial_port.write("Center point: ({}, {})".format(center_x, center_y).encode())
    serial_port.close()

    # 상황에 따라 온도모듈 선택해서 온도 특정
    if num_objects == 1:
        temperature = measure_temperature(objects)
        print("Temperature: {} ".format(temperature))
       
    else:
        # DHT11 
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: {}, Humidity: {}".format(temperature, humidity))


    # 온도 정보 통신하기
    serial_port = serial.Serial('/dev/ttyS0', 115200)
    serial_port.write("Temperature: {} ".format(temperature).encode())
    serial_port.close()

     #핀 초기화
    GPIO.cleanup()


if __name__ == "__main__":
    while True:
        detect_and_measure_temperature()
        time.sleep(1)




