import RPi.GPIO as GPIO
import time
import math
import ctypes

# 핀 번호 설정
ENA = 18  # 모터 A의 enable 핀 (PWM 제어)
IN1 = 23  # 모터 A의 입력 1


# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)


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

# PWM 객체 생성
pwm = GPIO.PWM(ENA, 100)  # 100Hz의 PWM 주파수

# 모터 제어 함수
def motor_control(direction, speed):
    GPIO.output(IN1, direction)
    pwm.start(speed)

# 모터 정지
def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    
    pwm.stop()

# 모터 속도를 계산하는 함수
def calculate_speed(temperature, distance):
    wind_speed = 0.35* abs(36-temperature)*math.pow(distance,0.5)
    return wind_speed


try:
    while True:
        temperature_lib = Temperature(libPath='./temperature.so')
        temperature_lib.check()
        result = temperature_lib.get_result()  # 결과 값 얻기
        temperature_value = result.replace("Object : ", "")
        print(result)
        speed = calculate_speed(int(float(result)),200)
        if(speed>=0.1): speed=0.1
        print("속도  : ",speed*1000)
        motor_control(GPIO.HIGH, speed * 1000)
        time.sleep(5)  

      
except KeyboardInterrupt:
    stop_motor()
    GPIO.cleanup()




