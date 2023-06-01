
import cv2
import numpy as np
import time

# 서보모터 관련 설정

# import RPi.GPIO as GPIO
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(18, GPIO.OUT)
# GPIO.setup(23, GPIO.OUT)
# p1 = GPIO.PWM(18, 50)   
# p2 = GPIO.PWM(23, 50)   
nowAngle_x = 0
nowAngle_y = 0
# p1.start(nowAngle_x)             
# p2.start(nowAngle_y)    
         
spinspeed = 5

def move_servo_x(angle):
    # 각도를 듀티 사이클로 변환
    duty_cycle = angle / 18 + 2
    # 듀티 사이클 변경
    #p1.ChangeDutyCycle(duty_cycle)

def move_servo_y(angle):
    # 각도를 듀티 사이클로 변환
    duty_cycle = angle / 18 + 2
    # 듀티 사이클 변경
    #p2.ChangeDutyCycle(duty_cycle)

# YOLOv3-tiny 모델을 위한 설정
net = cv2.dnn.readNet("yolov2-tiny.weights", "yolov2-tiny.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayersNames()

# 카메라 객체 생성
cap = cv2.VideoCapture(0)
wp = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
hp = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

point_x = [int(wp * 0.45), int(wp * 0.55)]
point_y = [int(hp * 0.45), int(hp * 0.55)]

def isInMiddle(point): #point = 튜플로, 중앙에 있는지 체크
    if(point[0] >= point_x[0] and point[0] <= point_x[1]):
        if(point[1] >= point_y[0] and point[1] <= point_y[1]):
            return True
    return False

def move_Servo_middle(point): #중앙으로 이동하게 회전
    global nowAngle_x
    global nowAngle_y
    if(point[0] < point_x[0]):
        print('left')
        nowAngle_x -= spinspeed
    elif point[0] > point_x[1]:
        print('right')
        nowAngle_x += spinspeed

    if(point[1] < point_y[0]):
        print('up')
        nowAngle_y -= spinspeed
    elif(point[1] > point_y[1]):
        print('down')
        nowAngle_y += spinspeed


while True:
    # 카메라에서 이미지 얻기
    ret, frame = cap.read()
    if not ret:
        break

    # 이미지 전처리
    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

    # YOLOv3-tiny 모델로 객체 감지
    net.setInput(blob)
    outs = net.forward(output_layers)

    # 감지된 객체 정보 저장
    class_ids = []
    confidences = []
    boxes = []
    centers = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # 객체가 발견되었을 때
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # 객체 바운딩 박스 좌표 계산
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)
                centers.append((center_x, center_y))

    # Non-maximum suppression 적용하여 중복 박스 제거
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)


    # 감지된 객체 정보 시각화
    colors = np.random.uniform(0, 255, size=(len(classes), 3))
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.line(frame, (point_x[0], 0), (point_x[0], hp), (255,0,0))
    cv2.line(frame, (point_x[1], 0), (point_x[1], hp), (255,0,0))
    cv2.line(frame, (0, point_y[0]), (wp, point_y[0]), (255,0,0))
    cv2.line(frame, (0, point_y[1]), (wp, point_y[1]), (255,0,0))

    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            
            
            # 감지된 객체가 사람이나 동물일 경우, 서보모터를 조작
            if label in ['person', 'dog', 'cat']: # 예시로 'person', 'dog', 'cat'으로 설정함
                confidence = str(round(confidences[i], 2))
                color = colors[class_ids[i]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                center_x = int(x + w/2)
                center_y = int(y + h/2)
                cv2.putText(frame, label + " " + confidence, (x, y + 20), font, 0.5, color, 2)
                cv2.circle(frame, (center_x, center_y), 5, color, -1)
                cv2.putText(frame, f'({center_x}, {center_y})', (center_x + 10, center_y + 10), font, 0.5, color, 2)

                # 감지된 객체가 2명 이상인 경우, 서보모터 1, 2의 각도 값 계산
                print(len(centers))
                if len(centers) > 1:
                    x_coordinates = [c[0] for c in centers]
                    y_coordinates = [c[1] for c in centers]
                    min_x, max_x = min(x_coordinates), max(x_coordinates)
                    min_y, max_y = min(y_coordinates), max(y_coordinates)
                    box_center_x = int((min_x + max_x) / 2)
                    box_center_y = int((min_y + max_y) / 2)
                    # 서보모터 1, 2의 각도 값 계산
                    nowAngle_x = int((box_center_x / width) * 180) # 0~180도 범위
                    nowAngle_y = int((box_center_y / height) * 180) # 0~180도 범위
                    # 서보모터 1, 2 움직임

                    # 1명인 경우, 대상의 중심을 카메라에 중심쪽으로 오게 회전
                elif len(centers) == 1:
                    print('객체 하나')
                    if(isInMiddle((center_x, center_y)) == False):
                        move_Servo_middle((center_x,center_y))
                    else:
                        print('중심에 있음')
                    
                move_servo_x(nowAngle_x)
                move_servo_y(nowAngle_y)
                   

    # 화면에 출력
    #cv2.imshow("Image", frame)
    #print(hp)
    #if cv2.waitKey(1) == ord('q'):
    #    break


cap.release()
cv2.destroyAllWindows()