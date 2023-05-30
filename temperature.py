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

def setup_dts():
    GPIO.setmode(GPIO.BCM)
    for pin in DTSPin.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    GPIO.output(DTSPin['SCK'], GPIO.HIGH)

def read_temperature_dts():
    data = 0
    for _ in range(16):
        GPIO.output(DTSPin['SCK'], GPIO.LOW)
        bit = GPIO.input(DTSPin['SDI'])
        data = (data << 1) | bit
        GPIO.output(DTSPin['SCK'], GPIO.HIGH)

    if data & 0x8000:
        temperature = -((data ^ 0xFFFF) + 1) / 16.0  # 음수 온도인 경우
    else:
        temperature = data / 16.0  # 양수 온도인 경우

    return temperature

if __name__ == "__main__":
    GPIO.setwarnings(False)
    try:
        setup_dts()
        while True:
            # DHT11
            humidity, temperature_dht = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
            print("Temperature (DHT11): {}, Humidity: {}".format(temperature_dht, humidity))

            # DTS-L300-V2
            temperature_dts = read_temperature_dts()
            print("Temperature (DTS-L300-V2): {}".format(temperature_dts))

            time.sleep(1)
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        GPIO.cleanup()
