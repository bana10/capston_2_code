import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT

ser = serial.Serial('/dev/ttyS0', 115200) # 시리얼 포트 설정



# 객체 감지 함수
def detect_objects(image):
    # YOLOv3-tiny 가중치 파일과 설정 파일 경로
    weights_path = "yolov3-tiny.weights"
    config_path = "yolov3-tiny.cfg"

    # 객체 라벨 리스트
    labels = ["person", "cat", "dog"]

    # 네트워크 불러오기
    net = cv2.dnn.readNetFromDarknet(config_path, weights_path)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # 이미지 전처리 및 객체 감지
    img = cv2.resize(image, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # 감지된 객체 리스트 초기화
    detected_objects = []

    # 감지된 객체 추출
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5 and labels[class_id] in labels:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                detected_objects.append((center_x, center_y))

    # 감지된 객체 반환
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

def detect_and_measure_temperature():
    # 카메라에서 이미지 캡처
    cap = cv2.VideoCapture(0)
    print("카메라 작동용")
    ret, frame = cap.read()
    cap.release()

    # 객체 감지
    objects = detect_objects(frame)
    print("객체 감지했다용")

    # 객체 수에 따라 온도 측정 방법 선택
    if len(objects) > 1:
        # DHT11 센서로 온도 측정
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("온도: %d , 습도: %d ".format(temperature,humidity))
    else:
        # DTS-L300-V2 센서로 온도 측정
        for i in range(6):
            GPIO.output(DTSPin[i], GPIO.HIGH)
        GPIO.output(DTSPin[0], GPIO.LOW)
        count = 0
        for i in range(6):
            GPIO.output(DTSPin[i], GPIO.LOW)
        while GPIO.input(DTSPin[5]) == GPIO.LOW:
            pass
        while GPIO.input(DTSPin[5]) == GPIO.HIGH:
            pass
        for i in range(32):
            GPIO.output(DTSPin[0], (i & 0x01))
            GPIO.output(DTSPin[1], (i & 0x02))
            GPIO.output(DTSPin[2], (i & 0x04))
            GPIO.output(DTSPin[3], (i & 0x08))
            GPIO.output(DTSPin[4], (i & 0x10))
            while GPIO.input(DTSPin[5]) == GPIO.LOW:
                pass
            count <<= 1
            if GPIO.input(DTSPin[5]) == GPIO.HIGH:
                count |= 0x01

        # 계산된 온도 값 반환
        temperature = (count * 0.0625)
        humidity = None
        print("온도 : %d".format(temperature))

    # 중심점 좌표 전송
    if len(objects) > 0:
        center_x, center_y = objects[0]
        message = "{},{}\n".format(center_x, center_y)
        ser.write(message.encode()) # 시리얼 포트로 중심점 좌표를 전송

    #핀 초기화
    GPIO.cleanup()
    # 온도 값 반환
    return temperature, humidity

if __name__ == "__main__":
    while True:
        detect_and_measure_temperature()
        time.sleep(1)




