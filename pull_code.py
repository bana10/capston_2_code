import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import board
import busio
import Adafruit_DHT
from ctypes import cdll, c_char_p
import math
import joblib
import sklearn

ser = serial.Serial('/dev/ttyS0', 115200) # 시리얼 포트 설정

DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(DHT_PIN, GPIO.IN)





class Temperature(object):
    def __init__(self, libPath):
        self.lib = cdll.LoadLibrary(libPath)
        self.obj = self.lib.Temperature_new()
        
    def check(self, tick=0.5):
        self.lib.Temperature_check(self.obj)
        result = self.get_result()
        temperature, object_value = map(float, result.split(", "))
        print("Temperature: {}, Object: {}".format(temperature, object_value))
        time.sleep(tick)
        
    def get_result(self):
        self.lib.Temperature_get_result.restype = c_char_p
        result = self.lib.Temperature_get_result(self.obj)
        return result.decode()


        
def detect_objects(frame):
    # YOLO configuration
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    objects = []
    
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            class_name = classes[class_id]
            
            if confidence > 0.5 and class_name in ["person", "cat", "dog"]:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                obj = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h
                }
                
                objects.append(obj)
    print("objects:")
    print(objects)
    # 객체 순서대로 반환
    return objects


def measure_temperature(obj_class):
    if obj_class in ["person", "cat", "dog"]:
        f = Temperature(libPath='./temperature.so')
        result = f.get_result()  # 결과 값 얻기
        result = result.replace("Sensor: ", "").replace("Object: ", "")
        result = result.strip()  # 문자열 양쪽의 공백 제거
        temperature, object_value = map(float, result.split(", "))
        object_value = object_value.split(":")[-1].strip()

        return object_value


def detect_and_measure_temperature():
    # Camera operation
    cap = cv2.VideoCapture(0)
    print("Camera initialized")
    ret, frame = cap.read()
    cap.release()

    # Object detection
    objects = detect_objects(frame)
    print("Objects detected")

    # Display frame
    cv2.imshow("Camera View", frame)

    # Extract center points
    center_points = []
    num_objects = len(objects)
    for obj in objects:
        center_x = obj["x"] + obj["w"] / 2
        center_y = obj["y"] + obj["h"] / 2
        center_points.append((center_x, center_y))
    
    print("center_points :")
    print(center_points)
    
    # 상황에 따라 온도모듈 선택해서 온도 특정
    if num_objects == 1:
        obj_class = objects[0]["class_name"]
        print(obj_class)
        temperature = measure_temperature(obj_class)
        print("Temperature: {} ".format(temperature))
    else:
        # DHT11 
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: {}, Humidity: {}".format(temperature, humidity))

    # Release the capture and close the window
    cv2.destroyAllWindows()

    # 핀 초기화
    GPIO.cleanup()
    
    return center_points


prev_x = 0
prev_y = 0


# 서보모터 gpio
MOTOR_X = 18
MOTOR_Y = 19

# 선풍기 모터
MOTOR_PIN = 22

# PWM 주파수 설정
PWM_FREQUENCY = 50

# PWM 사이클 범위
DUTY_CYCLE_MIN = 2.5
DUTY_CYCLE_MAX = 12.5

# gpio 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_X, GPIO.OUT)
GPIO.setup(MOTOR_Y, GPIO.OUT)
GPIO.setup(MOTOR_PIN, GPIO.OUT)



pwm_x = GPIO.PWM(MOTOR_X, PWM_FREQUENCY)
pwm_y = GPIO.PWM(MOTOR_Y, PWM_FREQUENCY)
pwm = GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)
pwm_x.start(0)
pwm_y.start(0)
pwm.start(0)

x_model = joblib.load('./Xloc_model.pkl')
y_model = joblib.load('./Yloc_model.pkl')

def move_motors(x, y):
    # 각 서보 모터 위치에 대한 PWM 듀티 사이클 계산
    duty_cycle_x = ((x / 640) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN
    duty_cycle_y = ((y / 480) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN)) + DUTY_CYCLE_MIN

    # 서보 모터를 지정된 위치로 이동
    pwm_x.ChangeDutyCycle(duty_cycle_x)
    time.sleep(0.5)
    pwm_y.ChangeDutyCycle(duty_cycle_y)
    time.sleep(0.5)

def get_sorted_centers(centers):
    # 스택에 저장된 중심점 좌표를 y값 기준으로 오름차순 정렬하는 코드 구현
    sorted_centers = sorted(centers, key=lambda center: center[1])
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
    position_x = (duty_cycle_x - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * 180
    position_y = (duty_cycle_y - DUTY_CYCLE_MIN) / (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * 180

    return position_x, position_y

def calculate_distance(point1, point2):
    # 두 점 사이의 거리를 계산
    x1, y1 = point1
    x2, y2 = point2

    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return distance



# 거리정보 받아오는거
def get_distance_info():
    message = ser.readline().decode().strip()
    distance = int(message)

    return distance





# 모터 동작 시키기
def control_motor(temperature, distance):
    
    if distance < 120:
        wind_speed = math.sqrt(abs(36 - temperature) / (0.5 * 1005))
    else:
        wind_speed = math.sqrt(abs(36 - temperature) / (0.5 * 1005 * (1 - (80 / distance)**2)))

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
    while True:
        center = detect_and_measure_temperature()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        sorted_centers = get_sorted_centers(center)
            
        # 스택에 저장된 중심점 좌표 개수가 1개인 경우
        if len(sorted_centers) == 1:
            center_x, center_y = sorted_centers[0]
            #모델로 좌표예측 추가
            if (prev_y == prev_x and prev_x == 0) is False:
                dat = [[prev_x,prev_y,center_x,center_y]]
                p_x = int(x_model.predict(dat))
                p_y = int(y_model.predict(dat))
            
            move_motors(p_x, p_y)
            prev_x = center_x
            prev_y = center_y

        else:  # 2인 이상 감지
            if len(sorted_centers) > 1:
                min_center = sorted_centers[0]
                max_center = sorted_centers[-1]

                # 가장 외곽의 두 점의 위치가 변경되었는지 확인한다
                if ((min_center == center[0] and max_center == center[-1]) or (min_center == center[-1] and max_center == center[0])) != 1:
                    # 양 끝점이 정렬된 것과 다른 경우 x축은 정렬 전 배열의 각 끝점을 각도로 환산한 만큼,
                    # y축은 정렬 양 끝값 평균값
                    angle1 = calculate_angle(center[0])
                    angle2 = calculate_angle(center[-1])

                    x_distance = abs(center[-1][0] - center[0][0])
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
