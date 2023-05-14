import serial
import RPi.GPIO as GPIO
import time

ser = serial.Serial('/dev/ttyAMA0', 9600) # 시리얼 포트 설정



# Servo motor GPIO pin number
MOTOR_PIN = 18

# PWM frequency for the hs-311 motor
PWM_FREQUENCY = 50

# PWM duty cycle range for the hs-311 motor
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

# Initialize the GPIO module
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN, GPIO.OUT)

# Initialize the PWM object
pwm = GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)
pwm.start(0)

def move_motor(x, y):
    # Calculate the PWM duty cycle for each servo motor position
    duty_cycle_x = ((y / 640) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    duty_cycle_y = ((x / 480) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN

    # Move the servo motor to the specified position
    pwm.ChangeDutyCycle(duty_cycle_x)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(duty_cycle_y)
    time.sleep(0.5)



def get_sorted_centers(centers):
    # 스택에 저장된 중심점 좌표를 정렬하는 코드 구현
    sorted_centers = sorted(centers)
    return sorted_centers

def calculate_angle(center):
    # 중심점 좌표의 x값을 0부터 180까지의 값으로 변환
    angle = int((center[0] / 640) * 180)
    return angle

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyS0', 115200)

# 중심점 좌표를 저장하는 스택 초기화
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
                    # 가장 작은 값과 큰 값의 중앙값을 기준으로 헤더가 좌우로 움직임
                    median_center = sorted_centers[len(sorted_centers) // 2]
                    left_end, right_end = sorted_centers[0], sorted_centers[-1]
                    if median_center[0] < left_end[0]:
                        move_motor(calculate_angle(median_center), calculate_angle(left_end))
                    elif median_center[0] > right_end[0]:
                        move_motor(calculate_angle(right_end), calculate_angle(median_center))
                    else:
                        # 양 끝 좌표의 각도를 구하여 모터를 움직임
                        angle1 = calculate_angle(sorted_centers[0])
                        angle2 = calculate_angle(sorted_centers[-1])
                        move_motor(angle1, angle2)

            #핀 초기화
            GPIO.cleanup()


