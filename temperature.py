import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT

# DTS-L300-V2 센서 설정
DTSPin = {
    'SCK': 25,
    'SDI': 24,
    'SDD': 23,
    'SCE': 22
}

# DHT11 센서 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(DTSPin['SCK'], GPIO.OUT)
GPIO.setup(DTSPin['SDI'], GPIO.OUT)
GPIO.setup(DTSPin['SDD'], GPIO.IN)
GPIO.setup(DTSPin['SCE'], GPIO.OUT)


def read_temperature():
    # 센서에서 데이터 읽기
    data = 0

    GPIO.output(DTSPin['SCE'], GPIO.LOW)

    for i in range(16):
        GPIO.output(DTSPin['SCK'], GPIO.HIGH)
        time.sleep(0.001)
        bit = GPIO.input(DTSPin['SDD'])
        data |= (bit << (15 - i))
        GPIO.output(DTSPin['SCK'], GPIO.LOW)
        time.sleep(0.001)

    GPIO.output(DTSPin['SCE'], GPIO.HIGH)

    # 온도 값 계산
    temp = ((data & 0xFF00) >> 8) + ((data & 0x00FF) / 256.0)
    temp = temp * 0.0625

    return temp

if __name__ == "__main__":
    while True:
        # DHT11
        humidity, temperature_dht = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature (DHT11): {}, Humidity: {}".format(temperature_dht, humidity))

        # DTS-L300-V2
        temperature_dts = read_temperature()
        print("Temperature: {:.2f} °C".format(temperature_dts))
        time.sleep(1)

    GPIO.cleanup()

