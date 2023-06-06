import numpy as np
import cv2
import time
import RPi.GPIO as GPIO
import Adafruit_DHT
import ctypes
import math


cap = cv2.VideoCapture(0)
prevt = time.time()
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# 핀 번호 설정
ENA = 18  # 모터 A의 enable 핀 (PWM 제어)
IN1 = 23  # 모터 A의 입력 1

x_servo_pin = 13 
y_servo_pin = 19
DHT_PIN = 12
DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(x_servo_pin, GPIO.OUT)
GPIO.setup(y_servo_pin, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)

x_pwm = GPIO.PWM(x_servo_pin, 50) 
y_pwm = GPIO.PWM(y_servo_pin, 50) 
tpwm = GPIO.PWM(ENA, 100)  # 100Hz의 PWM 주파수

x_pwm.start(0)
y_pwm.start(0)

def X_Motor(angle):
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5 
    x_pwm.ChangeDutyCycle(duty_cycle)

def Y_Motor(angle):
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5 
    y_pwm.ChangeDutyCycle(duty_cycle)


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
    

def measure_temperature(temperature_lib):
    temperature_lib.check()
    result = temperature_lib.get_result()  # 결과 값 얻기
    result = result.replace("Object : ", "")
    print(result)

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

wp = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
hp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

Xspin = 45
Yspin = 20

goleft = True

X_Motor(0)
Y_Motor(0)

temperature_lib = Temperature(libPath='./temperature.so')

distance = 25

try:
    while(True):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        boxes, weights = hog.detectMultiScale(gray, winStride=(8,8))
        maxx = 0
        minx = wp
        maxy = 0
        miny = hp

        if len(boxes) > 0:
            temperature = 10
            if len(boxes) == 1:
                temperature = measure_temperature(temperature_lib)
                print("Temperature: {} ".format(temperature))
            else:
                humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
                print("Temperature: {}, Humidity: {}".format(temperature, humidity))
            
            speed = calculate_speed(temperature, distance)
            print("속도  : %f", speed*1000)
            motor_control(GPIO.HIGH, speed * 1000)
            for (x,y,w,h) in boxes:
                cv2.rectangle(frame, (x,y), (x + w, y + h), (50,200,50), 2)
                center_x = x + int(w / 2)
                center_y = y + int(h / 2)

                if center_x > maxx:
                    maxx = center_x
                if center_x < minx:
                    minx = center_x
                if center_y > maxy:
                    maxy = center_y
                if center_y < miny:
                    miny = center_y

            minanglex = int((minx / wp * (Xspin * 2)) - Xspin)
            maxanglex = int((maxx / wp * (Xspin * 2)) - Xspin)

            minangley = int((miny / hp * (Yspin * 2)) - Yspin)
            maxangley = int((maxy / hp * (Yspin * 2)) - Yspin)

            if goleft:
                X_Motor(minanglex)
                print("goleft:", minanglex)
            else:
                X_Motor(maxanglex)
                print("goright:", maxanglex)

            yangleavg = int((minangley + maxangley) / 2)
            Y_Motor()
            print("avgy:", yangleavg)
            goleft = not goleft

        cv2.imshow('asdf',frame)
        if cv2.waitKey(1) == ord('q'):
            GPIO.cleanup()
            break

        print(time.time() - prevt)
        prevt = time.time()
        print(len(boxes))

except KeyboardInterrupt:
    GPIO.cleanup()