import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT

ser = serial.Serial('/dev/ttyS0', 115200) # 시리얼 포트 설정

'''
기존 코드에서..두번째 보드 코드 짜다가 카메라위치를 (0.0)으로 잡고 짜야되는 부분이 생겼어. 
내 머리로 짜기엔 토할거 같아서 chat gpt돌림.
이 코드는 카메라 위치를 (0.0)으로 잡고 객체의 중심점들을 추출하는 코드.
'''

# 객체 감지 함수
def detect_objects(image):
    #yolo3로 할 경우 
    weights_path = "yolov3-tiny.weights"
    config_path = "yolov3-tiny.cfg"

    # 임의로 설정한 감지 대상
    labels = ["person", "cat", "dog"]

    
    net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # image preprocessing and object detection
    img = cv2.resize(image, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # Initialize the list of detected objects
    detected_objects = []

    # Extract detected objects
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and labels[class_id] in labels:
                center_x = int(detection[0] * width) - int(width/2)
                center_y = int(detection[1] * height) - int(height/2)
                detected_objects.append((center_x, center_y))

    # return the detected objects
    return detected_objects



# DTS-L300-V2 센서 설정
DTSPin = [27, 22, 23, 24, 25, 26]
'''
SCK : GPIO 핀 25
SDI : GPIO 핀 24
SDD : GPIO 핀 23
SCE : GPIO 핀 22
'''


# DHT11 센서 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11


def read_temperature(DTSPin):
    GPIO.setmode(GPIO.BCM)
    for i in range(6):
        GPIO.setup(DTSPin[i], GPIO.OUT)
        GPIO.output(DTSPin[i], GPIO.LOW)

    # SCK 핀을 HIGH로 설정
    GPIO.output(DTSPin[4], GPIO.HIGH)

    # 16비트 데이터 읽기
    data = 0
    for i in range(16):
        # SCK 핀을 LOW로 설정
        GPIO.output(DTSPin[4], GPIO.LOW)

        # SDD 핀에서 데이터 읽기
        bit = GPIO.input(DTSPin[3])

        # 데이터를 1비트씩 왼쪽으로 시프트하고 읽은 비트를 맨 오른쪽에 추가
        data = (data << 1) | bit

        # SCK 핀을 HIGH로 설정
        GPIO.output(DTSPin[4], GPIO.HIGH)

    # 핀 초기화
    GPIO.cleanup()

    # 온도 계산
    if data & 0x8000:
        # 음수 온도인 경우
        temperature = -((data ^ 0xFFFF) + 1) / 16.0
    else:
        # 양수 온도인 경우
        temperature = data / 16.0

    return temperature



def detect_and_measure_temperature():
    # Capture image from camera
    cap = cv2.VideoCapture(0)
    print("For camera operation")
    ret, frame = cap.read()
    cap.release()

    # object detection
    objects = detect_objects(frame)
    print("object detected")

    # Calculate center point of detected object(s)
    center_x = 0
    center_y = 0
    num_objects = len(objects)
    for obj in objects:
        center_x += obj["x"] + obj["w"] / 2
        center_y += obj["y"] + obj["h"] / 2
    if num_objects > 0:
        center_x /= num_objects
        center_y /= num_objects

    # Send center point to serial port
    serial_port = serial.Serial('/dev/ttyUSB0', 9600)
    serial_port.write("Center point: ({}, {})".format(center_x, center_y).encode())
    serial_port.close()

    # Choose the temperature measurement method according to the number of objects
    if num_objects > 1:
        # Measure temperature with DHT11 sensor
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: %d , Humidity: %d ".format(temperature,humidity))
    else:
        # Measure temperature with DTS-L300-V2 sensor
        temperature = read_temperature(DTSPin)
        print("Temperature: %f" % temperature)

     #핀 초기화
    GPIO.cleanup()


if __name__ == "__main__":
    while True:
        detect_and_measure_temperature()
        time.sleep(1)




