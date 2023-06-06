import RPi.GPIO as GPIO
import time


servo_pin = 13 # GPIO pin connected to the servo motor
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

pwm = GPIO.PWM(servo_pin, 50) 
# 50 Hz PWM Frequency 이게 hs-311모터 기본 주파수. #데이터시트에 적혀있어
pwm.start(0)

def RC_Motor(angle):
    # angle : -90도 ~ +90도
    if angle < -90:
        angle = -90
    if angle > 90:
        angle = 90
    duty_cycle = (angle + 90) / 18 + 2.5
#듀티사이클을 왜 계산하면 gpio 라이브러리에서 서보모터 돌릴려면 ChangeDutyCycle 밖에 없어서 저럴 수 밖에 없어. 
#  (angle + 90)은 -90에서90도 값을 0에서 180으로 바꾼거고 
#18의 값은 서보 모터의 펄스 폭 범위와 스케일링 계수를 기준으로 선택됩니다. 예를 들어, 서보 모터의 회전 범위가 -90~90도일 때 펄스 폭 범위가 600~2400마이크로초인 경우, 이는 50Hz PWM 신호에 대해 약 3~12%의 듀티 사이클 범위에 해당합니다. 각도 값을 18로 나누면 이 듀티 사이클 범위로 스케일이 조정된대.  왜 이런 말투냐면 저기만 챗지피티 물어봄. 구하기 어려워서..
#2.5의 값은 서보 모터의 펄스 폭 범위와 스케일링 계수를 기준으로 선택됩니다. 예를 들어, 서보 모터의 회전 범위가 -90~90도일 때 펄스 폭 범위가 600~2400마이크로초인 경우, 이는 50Hz PWM 신호에 대해 약 3~12%의 듀티 사이클 범위에 해당합니다. 스케일링된 각도 값에 2.5를 더하면 최소 듀티 사이클이 3%로 설정됩니다 래. 근데 블로그 설명에서 보면 저거 펄스폭 범위가 600~2400마이크로초야. 
    pwm.ChangeDutyCycle(duty_cycle)


while True:
    RC_Motor(-90)
    time.sleep(0.5)
    RC_Motor(-45)
    time.sleep(0.5)
    RC_Motor(0)
    time.sleep(0.5)
    RC_Motor(45)
    time.sleep(0.5)
    RC_Motor(90)
    time.sleep(0.5)

