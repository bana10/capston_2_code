import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT




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

    # 온도 계산
    if data & 0x8000:
        # 음수 온도인 경우
        temperature = -((data ^ 0xFFFF) + 1) / 16.0
    else:
        # 양수 온도인 경우
        temperature = data / 16.0

    return temperature


if __name__ == "__main__":
    while True:
        # DHT11 
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: %d , Humidity: %d \n".format(temperature,humidity))

        # DTS-L300-V2 
        temperature = read_temperature(DTSPin)
        print("Temperature: %f\n" % temperature)

        time.sleep(1)