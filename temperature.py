import serial
import time
import cv2
import numpy as np
import RPi.GPIO as GPIO
import Adafruit_DHT
import board
import busio
import adafruit_mlx90614




# DHT11 센서 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

# MLX90614 센서 설정
i2c_bus = busio.I2C(board.SCL, board.SDA)
mlx = adafruit_mlx90614.MLX90614(i2c_bus)

def measure_temperature():
    # Measure the object's surface temperature using MLX90614
    infrared_temperature = mlx.object_temperature
    air_temperature, air_humidity = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
    surface_temperature = infrared_temperature + (air_temperature - infrared_temperature) * (100 - air_humidity) / 100
    return surface_temperature



if __name__ == "__main__":
    while True:
        temperature = measure_temperature()
        print("Surface Temperature: {}".format(temperature))
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        print("Temperature: {}, Humidity: {}".format(temperature, humidity))
        time.sleep(1)
        GPIO.cleanup()


