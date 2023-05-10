import cv2
import numpy as np
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



# YOLOv3-tiny 모델을 위한 설정
net = cv2.dnn.readNet("yolov2-tiny.weights", "yolov2-tiny.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = net.getUnconnectedOutLayersNames()

# 카메라 객체 생성
cap = cv2.VideoCapture(-1)

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
    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = str(round(confidences[i], 2))
            color = colors[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            center_x = int(x + w/2)
            center_y = int(y + h/2)
            cv2.putText(frame, label + " " + confidence, (x, y + 20), font, 0.5, color, 2)
            cv2.circle(frame, (center_x, center_y), 5, color, -1)
            cv2.putText(frame, f'({center_x}, {center_y})', (center_x + 10, center_y + 10), font, 0.5, color, 2)

            # center_x, center_y 값을 각도로 변환하여 PWM 신호 생성
            angle_x = center_x * 180 / 640  # 0~640 범위를 0~180도 범위로 변환
            angle_y = center_y * 180 / 480  # 0~480 범위를 0~180도 범위로 변환
            duty_x = min_duty + (max_duty - min_duty) * angle_x / 180
            duty_y = min_duty + (max_duty - min_duty) * angle_y / 180


            # 서보모터를 움직임
            pwm_x.start(duty_x)
            pwm_y.start(duty_y)


    # 화면에 출력
    cv2.imshow("Image", frame)
    if cv2.waitKey(1) == ord('q'):
        break




cap.release()
cv2.destroyAllWindows()

