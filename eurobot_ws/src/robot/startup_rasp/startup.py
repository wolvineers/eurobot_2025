import RPi.GPIO as GPIO
import time

pin_sensor = None

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def sensor_callback(channel):
    if GPIO.input(pin_sensor) == GPIO.LOW:
        print("Start")

GPIO.add_event_detect(pin_sensor, GPIO.BOTH, callback=sensor_callback, bouncetime=200)

try:
    while True:
        time.sleep(1)
    
except KeyboardInterrupt:
    GPIO.cleanup()