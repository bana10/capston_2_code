import RPi.GPIO as GPIO
import math
import time

# 선풍기 모터
MOTOR_PIN = 22

# PWM 주파수 설정
PWM_FREQUENCY = 50

# PWM 사이클 범위
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

GPIO.setup(MOTOR_PIN, GPIO.OUT)
pwm = GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)
pwm.start(0)


# 모터 동작 시키기
def control_motor(temperature, distance):
    
    if distance < 120:
        wind_speed = math.sqrt((36 - temperature) / (0.5 * 1005))
    else:
        wind_speed = math.sqrt((36 - temperature) / (0.5 * 1005 * (1 - (80 / distance)**2)))

    # 모터 PWM 계산 
    pwm_duty_cycle = (wind_speed * 100) / 20

    # 모터 동작시키기
    if wind_speed > 0:
        GPIO.output(MOTOR_PIN, GPIO.HIGH)
    else:
        GPIO.output(MOTOR_PIN, GPIO.LOW)

    # 모터 속도 조절
    pwm.ChangeDutyCycle(pwm_duty_cycle)
   


        



if __name__ == "__main__":
        while(True):
            # 온도
            temperature = 37
            # 거리
            distance = 100

            # 모터 동작
            control_motor(temperature, distance)

            # 1초 간격
            time.sleep(1)
    