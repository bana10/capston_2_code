import numpy as np
import time
import RPi.GPIO as GPIO
#import Adafruit_DHT
import ctypes
import math

prevt = time.time()

# 핀 번호 설정
ENA = 18  # 모터 A의 enable 핀 (PWM 제어)
IN1 = 23  # 모터 A의 입력 1

# DHT_PIN = 12
# DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
tpwm = GPIO.PWM(ENA, 100)  # 100Hz의 PWM 주파수

class Temperature(object):
    def __init__(self, libPath):
        self.lib = ctypes.CDLL("./temperature.so")
        self.obj = self.lib.Temperature_new()

    def check(self):
        self.lib.Temperature_check(self.obj)

    def get_result(self):
        self.lib.Temperature_get_result.restype = ctypes.c_char_p
        result = self.lib.Temperature_get_result(self.obj)
        return result.decode()


def measure_temperature(temperature_lib):
    temperature_lib.check()
    result = temperature_lib.get_result()  # 결과 값 얻기

    return result


def motor_control(direction, speed):
    GPIO.output(IN1, direction)
    tpwm.start(speed)


def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    tpwm.stop()


def calculate_speed(temperature, distance):
    if distance < 120:
        wind_speed = math.sqrt(abs(36 - temperature) / (0.5 * 1005))
    else:
        wind_speed = math.sqrt(abs(36 - temperature) /
                               (0.5 * 1005 * (1 - (80 / distance)**2)))
    return wind_speed

temperature_lib = Temperature(libPath="./temperature.so")

distance = 150

try:
    while(True):
        temperature_lib.check()
        result = temperature_lib.get_result()  # 결과 값 얻기
        temperature_value = result.replace("Object : ", "")
        print(result)
        speed = calculate_speed(int(float(result)),200) *1000
        print("speed : ", speed)

        if speed >= 100:
            speed = 100
        elif speed <= 0:
            speed = 0

        motor_control(GPIO.HIGH, speed)
        time.sleep(0.5)

except KeyboardInterrupt:
    GPIO.cleanup()
