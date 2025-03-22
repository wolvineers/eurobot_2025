import RPi.GPIO as GPIO
import time
import subprocess
import os

BUTTON_PIN = 8
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:

    while True:
    
	    button_state = GPIO.input(BUTTON_PIN)
	    if (button_state == GPIO.HIGH):
		    print("Activat")
	    else:
		    print("No actiu")
	    time.sleep(0.1)

except Exception as e:
    print(f"ERROR {e}")

finally:
    GPIO.cleanup()
