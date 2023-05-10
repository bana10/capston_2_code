import RPi.GPIO as GPIO
import time



# 서보모터를 제어할 GPIO 핀 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)

# PWM 객체 생성
pwm_x = GPIO.PWM(18, 50)  # 18번 핀, 주기 50Hz
pwm_y = GPIO.PWM(19, 50)  # 19번 핀, 주기 50Hz

# 듀티 사이클 최솟값, 최댓값 설정
min_duty = 2.5
max_duty = 12.5

# center_x, center_y 값을 각도로 변환하여 PWM 신호 생성
angle_x = center_x * 180 / 640  # 0~640 범위를 0~180도 범위로 변환
angle_y = center_y * 180 / 480  # 0~480 범위를 0~180도 범위로 변환
duty_x = min_duty + (max_duty - min_duty) * angle_x / 180
duty_y = min_duty + (max_duty - min_duty) * angle_y / 180

# 서보모터를 움직임
pwm_x.start(duty_x)
pwm_y.start(duty_y)
