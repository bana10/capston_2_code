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
GPIO.setmode(GPIO.BCM)
for i in range(6):
    GPIO.setup(DTSPin[i], GPIO.OUT)
    GPIO.output(DTSPin[i], GPIO.LOW)

# DHT11 센서 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

# def detect_and_measure_temperature():
#     # 카메라에서 이미지 캡처
#     cap = cv2.VideoCapture(0)
#     print("카메라 작동용")
#     ret, frame = cap.read()
#     cap.release()

#     # 객체 감지
#     objects = detect_objects(frame)
#     print("객체 감지했다용")

#     # 객체 수에 따라 온도 측정 방법 선택
#     if len(objects) > 1:
#         # DHT11 센서로 온도 측정
#         humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
#         print("온도: %d , 습도: %d ".format(temperature,humidity))
#     else:
#         # DTS-L300-V2 센서로 온도 측정
#         for i in range(6):
#             GPIO.output(DTSPin[i], GPIO.HIGH)
#         GPIO.output(DTSPin[0], GPIO.LOW)
#         count = 0
#         for i in range(6):
#             GPIO.output(DTSPin[i], GPIO.LOW)
#         while GPIO.input(DTSPin[5]) == GPIO.LOW:
#             pass
#         while GPIO.input(DTSPin[5]) == GPIO.HIGH:
#             pass
#         for i in range(32):
#             GPIO.output(DTSPin[0], (i & 0x01))
#             GPIO.output(DTSPin[1], (i & 0x02))
#             GPIO.output(DTSPin[2], (i & 0x04))
#             GPIO.output(DTSPin[3], (i & 0x08))
#             GPIO.output(DTSPin[4], (i & 0x10))
#             while GPIO.input(DTSPin[5]) == GPIO.LOW:
#                 pass
#             count <<= 1
#             if GPIO.input(DTSPin[5]) == GPIO.HIGH:
#                 count |= 0x01

#         # 계산된 온도 값 반환
#         temperature = (count * 0.0625)
#         humidity = None
#         print("온도 : %d".format(temperature))

#     # 중심점 좌표 전송
#     if len(objects) > 0:
#         center_x, center_y = objects[0]
#         message = "{},{}\n".format(center_x, center_y)
#         ser.write(message.encode()) # 시리얼 포트로 중심점 좌표를 전송

#     #핀 초기화
#     GPIO.cleanup()
#     # 온도 값 반환
#     return temperature, humidity


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
        for i in range(6):
            GPIO.output(DTSPin[i], GPIO.HIGH)
        GPIO.output(DTSPin[0], GPIO.LOW)
        count = 0
        for i in range(6):
            GPIO.output(DTSPin[i], GPIO.LOW)
        while GPIO.input(DTSPin[5]) == GPIO.LOW:
            pass

     #핀 초기화
    GPIO.cleanup()


if __name__ == "__main__":
    while True:
        detect_and_measure_temperature()
        time.sleep(1)




