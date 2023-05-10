// 초음파 센서 3개 핀 번호 정의
#define TRIGGER_PIN_FRONT 2
#define ECHO_PIN_FRONT 3
#define TRIGGER_PIN_BACK_LEFT 4
#define ECHO_PIN_BACK_LEFT 5
#define TRIGGER_PIN_BACK_RIGHT 6
#define ECHO_PIN_BACK_RIGHT 7

// RC 카드 핀 번호 정의
#define MOTOR_LEFT_PIN 9
#define MOTOR_RIGHT_PIN 10
#define MOTOR_STOP 1500 // RC 카드 정지 값
#define MOTOR_FORWARD 1700 // RC 카드 전진 값
#define MOTOR_BACKWARD 1300 // RC 카드 후진 값

// HIGH는 정방향 회전(전진), LOW는 역방향 회전(후진)을 의미함.

// 거리(cm) 측정 함수
float measureDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2.0;

  return distance;
}

void setup() {
  // 초음파 센서 핀 모드 설정
  pinMode(TRIGGER_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIGGER_PIN_BACK_LEFT, OUTPUT);
  pinMode(ECHO_PIN_BACK_LEFT, INPUT);
  pinMode(TRIGGER_PIN_BACK_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_BACK_RIGHT, INPUT);

  // RC 카드 핀 모드 설정
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
}

void loop() {
  // 앞쪽 초음파 센서를 이용하여 거리 측정
  float distance_front = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);

  // 거리가 50cm 미만이면 RC 카드를 후진시켜 거리 확보
  if (distance_front < 50) {
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
  }
  
  // 뒤쪽 초음파 센서 중 거리가 가장 가까운 센서를 이용하여 거리 측정
  float distance_back_left = measureDistance(TRIGGER_PIN_BACK_LEFT, ECHO_PIN_BACK_LEFT);
  float distance_back_right = measureDistance(TRIGGER_PIN_BACK_RIGHT, ECHO_PIN_BACK_RIGHT);
  float distance_back = min(distance_back_left, distance_back_right);

  // 거리가 30cm 미만인 경우 RC 카드를 후진시켜 장애물 피하기
  if (distance_back < 30) {
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
  }

  // 거리가 충분히 확보된 경우 RC 카드 전진
  else {
    digitalWrite(MOTOR_LEFT_PIN, HIGH);
    digitalWrite(MOTOR_RIGHT_PIN, HIGH);
    delay(1000);
    digitalWrite(MOTOR_LEFT_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_PIN, LOW);
  }
}

