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

SPEED_1MHz = 1000000  # SPI Speed
SPI_MODE3 = 3  # SPI MODE
OBJECT = 0xA0  # COMMAND(Read Object Temp.)
SENSOR = 0xA1  # COMMAND(Read Sensor Temp.)

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = SPEED_1MHz
spi.mode = SPI_MODE3

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(DTSPin['SDD'], GPIO.OUT)  # gpio 23 핀을 출력으로 설정
GPIO.output(DTSPin['SDD'], 1)  # gpio 23 핀을 HIGH로 설정

def spi_command(adr):
    data_buf = [adr, 0x22, 0x22]

    GPIO.output(DTSPin['SDD'], 0)
    time.sleep(0.00001)

    resp = spi.xfer2(data_buf)

    GPIO.output(DTSPin['SDD'], 1)

    return (resp[2] * 256 + resp[1])

if __name__ == "__main__":
    while True:
        # DHT11
        humidity, temperature_dht = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature (DHT11): {}, Humidity: {}".format(temperature_dht, humidity))

        # DTS-L300-V2
        iSensor = spi_command(SENSOR)
        iObject = spi_command(OBJECT)
        print("Sensor: {:.2f}, Object: {:.2f}".format(iSensor/100, iObject/100))
        time.sleep(1)

        GPIO.cleanup()
