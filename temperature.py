import Adafruit_DHT
import time
import board
import busio
import adafruit_mlx90614
import RPi.GPIO as GPIO 

# GPIO 핀 번호 설정
DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11


i2c = busio.I2C(board.SCL, board.SDA)
mlx = adafruit_mlx90614.MLX90614(i2c)

# GPIO 핀 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(DHT_PIN, GPIO.IN)

def measure_temperature():
    infrared_temperature = mlx.object_temperature
    air_temperature, air_humidity = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
    surface_temperature = infrared_temperature + (air_temperature - infrared_temperature) * (100 - air_humidity) / 100
    return surface_temperature


if __name__ == "__main__":
    while True:
        humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
        if humidity is not None and temperature is not None:
            print("Temperature: {:.1f}°C, Humidity: {:.1f}%".format(temperature, humidity))
        else:
            print("Failed to retrieve data from DHT sensor.")
            
        temperature = measure_temperature()
        print("Surface Temperature: {}".format(temperature))
        time.sleep(1)







