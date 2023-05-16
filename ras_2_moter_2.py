import serial
import RPi.GPIO as GPIO
import time
import math

ser = serial.Serial('/dev/ttyS0', 115200) # 시리얼 포트 설정

# 서보모터 gpio
MOTOR_X = 18
MOTOR_Y = 19

# PWM 주파수 설정
PWM_FREQUENCY = 50

# PWM 사이클 범위
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

# gpio 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_X, GPIO.OUT)
GPIO.setup(MOTOR_Y, GPIO.OUT)


pwm_x = GPIO.PWM(MOTOR_X, PWM_FREQUENCY)
pwm_y = GPIO.PWM(MOTOR_Y, PWM_FREQUENCY)
pwm_x.start(0)
pwm_y.start(0)


def move_motors(x, y):
    # 각 서보 모터 위치에 대한 PWM 듀티 사이클 계산
    duty_cycle_x = ((x / 640) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    duty_cycle_y = ((y / 480) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN

    # 서보 모터를 지정된 위치로 이동
    pwm_x.ChangeDutyCycle(duty_cycle_x)
    time.sleep(0.5)
    pwm_y.ChangeDutyCycle(duty_cycle_y)
    time.sleep(0.5)

# def get_sorted_centers(centers):
#     # 스택에 저장된 중심점 좌표를 y값 기준으로 오름차순 정렬하는 코드 구현
#     sorted_centers = sorted(centers, key=lambda center: center[1])
#     return sorted_centers

# 
def get_sorted_centers(centers):
    def quick_sort(arr):
        if len(arr) <= 1:
            return arr
        pivot = arr[len(arr) // 2][1]
        left = [x for x in arr if x[1] < pivot]
        middle = [x for x in arr if x[1] == pivot]
        right = [x for x in arr if x[1] > pivot]
        return quick_sort(left) + middle + quick_sort(right)
    
    sorted_centers = quick_sort(centers)
    return sorted_centers


def calculate_angle(center):
    # 중심점 좌표의 x값을 0부터 180까지의 값으로 변환
    angle = int((center[0] / 640) * 180)
    return angle

def get_current_motor_position():
    # PWM 신호의 duty cycle을 이용하여 모터의 현재 위치를 계산하고 반환
    duty_cycle_x = pwm_x.get_duty_cycle()
    duty_cycle_y = pwm_y.get_duty_cycle()

    # 현재 위치를 duty cycle 값에서 위치로 변환하는 계산식을 사용하여 계산
    position_x = (duty_cycle_x - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * (max_x - min_x) + min_x
    position_y = (duty_cycle_y - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * (max_y - min_y) + min_y

    return position_x, position_y

def calculate_distance(point1, point2):
    # 두 점 사이의 거리를 계산
    x1, y1 = point1
    x2, y2 = point2

    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance


if __name__ == "__main__":
    # 중심점 좌표를 저장하는 스택 초기화
    centers = []

    while True:
        if ser.in_waiting != 0:
            # 시리얼 포트로부터 중심점 좌표를 읽어옴
            message = ser.readline().decode().strip()
            center_x, center_y = map(int, message.split(","))
            
            # 중심점 좌표를 스택에 저장
            centers.append((center_x, center_y))
            # 스택에 저장된 중심점 좌표를 정렬
            sorted_centers = get_sorted_centers(centers)
            
            # 스택에 저장된 중심점 좌표 개수가 1개인 경우
            if len(sorted_centers) == 1:
                #욱현이 파트
                pass

            else: #2인 이상 감지
                min_center = sorted_centers[0]
                max_center = sorted_centers[-1]
                # 가장 외곽의 두 점의 위치가 변경되었는지 확인한다
                if ((min_center == centers[0] and max_center == centers[-1]) 
                    or (min_center == centers[-1] and max_center == centers[0]))!=1:
                   # 양 끝점이 정렬된 것과 다른 경우 x축은 정렬전 배열의 각 끝점을 각도로 환산한 만큼, 
                   # y축은 정렬 양 끝값 평균값
                    angle1 = calculate_angle(centers[0])
                    angle2 = calculate_angle(centers[-1])
                    x_distance = abs(centers[-1][0] - centers[0][0])
                    x_angle = calculate_angle((0, 0), (x_distance, 0)) 
                    move_motors(angle1 + x_angle, (sorted_centers[0][1] + sorted_centers[-1][1]) / 2)

                else:
                    # 현재 모터의 위치
                    current_position = get_current_motor_position() 
                   # 가장 작은 점과 가장 큰 점 사이의 대각선을 따라 모터를 이동한다
                    median_center = sorted_centers[len(sorted_centers) // 2]
                    left_end, right_end = sorted_centers[0], sorted_centers[-1]

                    # 현재 위치와 가까운 쪽 방향으로 우선 대각선으로 이동
                    if calculate_distance(current_position, left_end) < calculate_distance(current_position, right_end):
                        angle1 = calculate_angle(current_position, left_end)
                        angle2 = calculate_angle(median_center, right_end)
                    else:
                        angle1 = calculate_angle(current_position, right_end)
                        angle2 = calculate_angle(median_center, left_end)

                    move_motors(angle1, angle2)

                    


                    """
                    1. sorted_centers 리스트에서 중앙에 있는 점인 median_center를 찾음. 

                    2. sorted_centers의 첫 번째와 마지막 점을 각각 left_end와 right_end에 할당.
                    
                    3. median_center의 x 좌표와 left_end의 x 좌표를 비교하여 각도를 계산합니다. 
                    만약 median_center의 x 좌표가 left_end의 x 좌표보다 작다면, 
                    angle1은 median_center와 left_end 사이의 각도가 되고, 
                    angle2는 median_center와 right_end 사이의 각도가 됩니다. 

                    그렇지 않으면 angle1은 median_center와 right_end 사이의 각도가 되고, 
                    angle2는 median_center와 left_end 사이의 각도가 됩니다.
                    
                    4. 계산된 angle1과 angle2를 이용하여 모터를 제어합니다. 
                    """
