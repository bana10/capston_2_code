import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT
import spidev


# DTS-L300-V2 센서 설정
DTSPin = {
    'SCK': 25,
    'SDI': 24,
    'SDD': 23,
    'SCE': 22
}

# DHT11 센서 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(DTSPin['SCE'], GPIO.OUT)
GPIO.output(DTSPin['SCE'], GPIO.HIGH)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000  # SPI 통신 속도 설정 (1MHz)

# DTS-L300-V2 센서 초기화 함수
def initialize_sensor():
    spi.xfer2([0x01])  # Initialization 명령어 전송
    time.sleep(0.05)  # 초기화 시간 대기

def read_temperature():
    # 센서에서 데이터 읽기
    spi.xfer2([0x01])  # Start Conversion 명령어 전송
    time.sleep(0.05)  # 변환이 완료될 때까지 대기

    spi.xfer2([0x00])  # Read Data 명령어 전송
    time.sleep(0.05)  # 데이터 읽기 대기

    resp = spi.xfer2([0x00, 0x00])  # 데이터 수신

    # 온도 값 계산
    if resp[0] & 0x8000:
        raw_temp = resp[0] - 0x8000
    else:
        raw_temp = resp[0]
    
    temp = (raw_temp * 0.02) - 273.15

    return temp


if __name__ == "__main__":
    initialize_sensor()
    while True:
        # DHT11
        humidity, temperature_dht = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature (DHT11): {}, Humidity: {}".format(temperature_dht, humidity))

        # DTS-L300-V2
        temperature_dts = read_temperature()
        print("Temperature: {:.2f} °C".format(temperature_dts))
        time.sleep(1)

    GPIO.cleanup()

