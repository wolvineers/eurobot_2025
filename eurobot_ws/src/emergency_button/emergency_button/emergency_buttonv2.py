import RPi.GPIO as GPIO
import time
from utils.scripts.serial_communication import open_serial_port, send_message

BUTTON_PIN = 40 
port01 = '/dev/ttyUSB0'
port02 = '/dev/ttyUSB1'
baudrate = 115200
serial_port01 = open_serial_port(port01, baudrate)
serial_port02 = open_serial_port(port02, baudrate)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            send_message(serial_port01, "STOP")
            send_message(serial_port02, "STOP")
            print("hola")
        time.sleep(0.1)
finally:
    GPIO.cleanup()
