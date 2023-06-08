import RPi.GPIO as GPIO
import time


servo_pin = 13  # GPIO pin connected to the servo motor
servo_pin2 = 19  # GPIO pin connected to the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(servo_pin2, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50)
pwm2 = GPIO.PWM(servo_pin2, 50)
# 50 Hz PWM Frequency 이게 hs-311모터 기본 주파수. #데이터시트에 적혀있어
pwm.start(0)
pwm2.start(0)


def RC_Motor(angle):
    # angle : -90도 ~ +90도
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5
    pwm.ChangeDutyCycle(duty_cycle)


def RC_Motor2(angle):
    # angle : -90도 ~ +90도
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5
    pwm2.ChangeDutyCycle(duty_cycle)


RC_Motor(-90)
time.sleep(3)
RC_Motor(90)
time.sleep(3)
RC_Motor(0)
time.sleep(3)
RC_Motor2(0)
time.sleep(1)
GPIO.cleanup()
