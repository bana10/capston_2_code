import RPi.GPIO as GPIO
import time
import math
import Adafruit_DHT

# 핀 번호 설정
ENA = 18  # 모터 A의 enable 핀 (PWM 제어)
IN1 = 23  # 모터 A의 입력 1
DHT_PIN = 12
DHT_TYPE = Adafruit_DHT.DHT11

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)

# PWM 객체 생성
pwm = GPIO.PWM(ENA, 100)  # 100Hz의 PWM 주파수

# 모터 제어 함수
def motor_control(direction, speed):
    GPIO.output(IN1, direction)
    pwm.start(speed)

# 모터 정지 함수
def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    pwm.stop()

# 모터 속도를 계산하는 함수
def calculate_speed(humidity, temperature, distance):
    wind_speed = 0.42 * (math.sqrt(temperature) - 23.8) * ((100 - humidity) / 10) * math.pow((distance / 10), -0.2)
    return wind_speed

try:
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: {}, Humidity: {}".format(temperature, humidity))
        speed = calculate_speed(humidity, temperature, 80)
        print(speed)
        print("속도: {}".format(speed * 100))
        motor_control(GPIO.HIGH, speed * 100)
        time.sleep(10)

except KeyboardInterrupt:
    stop_motor()
    GPIO.cleanup()

GPIO.cleanup()
