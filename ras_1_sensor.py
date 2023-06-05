import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT
import ctypes


DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(DHT_PIN, GPIO.IN)




class Temperature(object):
    def __init__(self, libPath):
        self.lib = ctypes.CDLL(libPath)
        self.obj = self.lib.Temperature_new()

    def check(self):
        self.lib.Temperature_check(self.obj)

    def get_result(self):
        self.lib.Temperature_get_result.restype = ctypes.c_char_p
        result = self.lib.Temperature_get_result(self.obj)
        return result.decode()


def detect_objects(frame, net, classes):
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
    print("objects:")
    print(objects)
    # 객체 순서대로 반환
    return objects


def measure_temperature(obj_class, temperature_lib):
    if obj_class in ["person", "cat", "dog"]:
        temperature_lib.check()
        result = temperature_lib.get_result()  # 결과 값 얻기
        temperature_value = result.replace("Object : ", "")
        print(result)
        
        return result










def detect_and_measure_temperature(cap, net, classes):
    # Read frame from camera
    ret, frame = cap.read()

    # Object detection
    objects = detect_objects(frame, net, classes)
    print("Objects detected")

 
    # Extract center points
    center_points = []
    num_objects = len(objects)
    for obj in objects:
        center_x = obj["x"] + obj["w"] / 2
        center_y = obj["y"] + obj["h"] / 2
        center_points.append((center_x, center_y))
    
    print("center_points :")
    print(center_points)
    
    # Temperature measurement
    temperature_lib = Temperature(libPath='./temperature.so')
    
    if num_objects == 1:
        obj_class = objects[0]["class_name"]
        print(obj_class)
        temperature = measure_temperature(obj_class, temperature_lib)
        print("Temperature: {} ".format(temperature))
    else:
        # DHT11 
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: {}, Humidity: {}".format(temperature, humidity))

   
    # 핀 초기화
    GPIO.cleanup()

    return center_points


if __name__ == "__main__":
    # Load YOLO and coco.names
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    # Camera operation
    cap = cv2.VideoCapture(0)
    print("Camera initialized")

    while True:
        detect_and_measure_temperature(cap, net, classes)
     
        time.sleep(0.5)

 
