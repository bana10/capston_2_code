import Adafruit_DHT
import time
import RPi.GPIO as GPIO

DHT_PIN = 18
DHT_TYPE = Adafruit_DHT.DHT11

GPIO.setmode(GPIO.BCM)
GPIO.setup(DHT_PIN, GPIO.IN)

def read_temperature_humidity():
    humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, DHT_PIN)
    if humidity is not None and temperature is not None:
        return temperature, humidity
    else:
        return None, None

if __name__ == "__main__":
    while True:
        temperature, humidity = read_temperature_humidity()
        if temperature is not None and humidity is not None:
            print("Temperature: {:.1f}°C, Humidity: {:.1f}%".format(temperature, humidity))
        else:
            print("Failed to retrieve data from DHT sensor.")
        time.sleep(1)
