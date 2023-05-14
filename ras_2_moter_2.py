import serial
import RPi.GPIO as GPIO
import time

ser = serial.Serial('/dev/ttyS0', 115200) # 시리얼 포트 설정

# Servo motor GPIO 핀 번호
MOTOR_PIN_X = 18
MOTOR_PIN_Y = 19

# hs-311 모터용 PWM 주파수
PWM_FREQUENCY = 50

# hs-311 모터용 PWM 듀티 사이클 범위
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

# GPIO 모듈 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN_X, GPIO.OUT)
GPIO.setup(MOTOR_PIN_Y, GPIO.OUT)

# PWM 객체 초기화
pwm_x = GPIO.PWM(MOTOR_PIN_X, PWM_FREQUENCY)
pwm_x.start(0)

pwm_y = GPIO.PWM(MOTOR_PIN_Y, PWM_FREQUENCY)
pwm_y.start(0)

def move_motor(x, y):
    # 각 서보 모터 위치에 해당하는 PWM 듀티 사이클 계산
    duty_cycle_x = ((y / 640) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    duty_cycle_y = ((x / 480) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    # 서보 모터를 지정된 위치로 이동
    pwm_x.ChangeDutyCycle(duty_cycle_x)
    time.sleep(0.5)
    pwm_y.ChangeDutyCycle(duty_cycle_y)
    time.sleep(0.5)

def get_sorted_centers(centers):
    # 저장된 중심점 좌표를 정렬하는 코드 구현
    sorted_centers = sorted(centers)
    return sorted_centers


def calculate_angle(center):
    # 중심점 좌표의 x값을 0부터 180까지의 값으로 변환
    angle = int((center[0] / 640) * 180)
    return angle


centers = []

if __name__ == "__main__":
    while True:
        if ser.in_waiting != 0:
            # 시리얼 포트로부터 중심점 좌표를 읽어옴
            message = ser.readline().decode().strip()
            center_x, center_y = message.split(",")
            center_x = int(center_x)
            center_y = int(center_y)

            # 중심점 좌표를 스택에 저장
            centers.append((center_x, center_y))

            # 스택에 저장된 중심점 좌표를 정렬
            sorted_centers = get_sorted_centers(centers)

            # 스택에 저장된 중심점 좌표 개수가 1개인 경우
            if len(sorted_centers) == 1:
                move_motor(center_x, center_y)

            else:
                # 가장 작은 중심점과 가장 큰 중심점의 위치 변화 확인
                min_center = sorted_centers[0]
                max_center = sorted_centers[-1]
                if min_center != sorted_centers[0] or max_center != sorted_centers[-1]:
                    # 양 끝 좌표의 각도를 구하여 모터를 움직임
                    angle1 = calculate_angle(sorted_centers[0])
                    angle2 = calculate_angle(sorted_centers[-1])
                    move_motor(angle1, angle2)
                else:
                    # 양 끝 좌표가 변하지 않았을 경우에는 이전 좌표를 그대로 사용
                    move_motor(angle1, angle2)

