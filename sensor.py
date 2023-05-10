# 적외선이랑 주변온도 같이 측정가능한 센서 이용
import RPi.GPIO as GPIO
import time
#DHT11모듈 코드
import Adafruit_DHT

# 핀 번호 상수 정의
SCE_PIN = 18 
SCK_PIN = 11 
SDI_PIN = 8 

# DHT11 모듈이 연결된 GPIO 핀 번호 설정
DHT_PIN = 7 


GPIO.setmode(GPIO.BCM)
GPIO.setup(SCE_PIN, GPIO.OUT)
GPIO.setup(SCK_PIN, GPIO.OUT)
GPIO.setup(SDI_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.output(SCE_PIN, GPIO.HIGH)
GPIO.output(SCK_PIN, GPIO.LOW)


def read_temp():
    # 센서에서 온도 값 읽어오기
    cmd = bytearray.fromhex('01 03 00 00 00 01 84 0A')
    result = None
    try:
        # SPI 통신 시작
        GPIO.output(SCE_PIN, GPIO.LOW)
        for c in cmd:
            for i in range(8):
                if c & 0x80:
                    GPIO.output(SDI_PIN, GPIO.HIGH)
                else:
                    GPIO.output(SDI_PIN, GPIO.LOW)
                c <<= 1
                GPIO.output(SCK_PIN, GPIO.HIGH)
                GPIO.output(SCK_PIN, GPIO.LOW)
        # 데이터 수신
        data = bytearray()
        for i in range(7):
            byte = 0
            for j in range(8):
                GPIO.output(SCK_PIN, GPIO.HIGH)
                bit = GPIO.input(SDI_PIN)
                byte <<= 1
                byte |= bit
                GPIO.output(SCK_PIN, GPIO.LOW)
            data.append(byte)
        # 센서로부터 받은 데이터 처리
        if data[0] == 0x01 and data[1] == 0x03 and data[2] == 0x02:
            temp = (data[3] << 8) + data[4]
            result = temp / 50.0
    finally:
        # SPI 통신 종료
        GPIO.output(SCE_PIN, GPIO.HIGH)
    return result

if __name__ == '__main__':
    try:
        while True:
            temp = read_temp()
            # DHT11 모듈에서 온도 및 습도 측정
            humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, DHT_PIN)

            # 측정한 온도 및 습도 출력
            if humidity is not None and temperature is not None:
                print('대기온도 ={0:0.1f}*C  습도 ={1:0.1f}%'.format(temperature, humidity))
            else:
                print('Failed to get reading from the sensor. Check wiring.')
            
            # 적외선으로 온도 측정하는거
            if temp is not None:
                print('온도 :', temp)
            else:
                print('Failed to read temperature')
            time.sleep(1)
    finally:
        GPIO.cleanup()







