import RPi.GPIO as GPIO
import math
import time

# 모터 드라이브
MOTOR_PIN_ENA = 17
MOTOR_PIN_IN1 = 27
MOTOR_PIN_IN2 = 22

# PWM 주파수 설정
PWM_FREQUENCY = 100

# PWM 사이클 범위
DUTY_CYCLE_MIN = 0
DUTY_CYCLE_MAX = 100

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_ENA, GPIO.OUT)
GPIO.setup(MOTOR_PIN_IN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN_IN2, GPIO.OUT)

pwm = GPIO.PWM(MOTOR_PIN_ENA, PWM_FREQUENCY)
pwm.start(0)


# 모터 동작 시키기
def control_motor(temperature, distance):
    if distance < 120:
        wind_speed = math.sqrt(abs(36 - temperature) / (0.5 * 1005))
    else:
        wind_speed = math.sqrt(abs(36 - temperature) / (0.5 * 1005 * (1 - (80 / distance)**2)))

    # 모터 속도 계산
    duty_cycle = (wind_speed / 20) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) + DUTY_CYCLE_MIN

    # 모터 방향 설정
    if wind_speed > 0:
        GPIO.output(MOTOR_PIN_IN1, GPIO.HIGH)
        GPIO.output(MOTOR_PIN_IN2, GPIO.LOW)
    else:
        GPIO.output(MOTOR_PIN_IN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN_IN2, GPIO.LOW)

    # 모터 속도 조절
    pwm.ChangeDutyCycle(duty_cycle)


if __name__ == "__main__":
    try:
        while True:
            # 온도
            temperature = 37
            # 거리
            distance = 100

            # 모터 동작
            control_motor(temperature, distance)

            # 1초 간격
            time.sleep(1)

    except KeyboardInterrupt:
        pass

    finally:
        GPIO.cleanup()
