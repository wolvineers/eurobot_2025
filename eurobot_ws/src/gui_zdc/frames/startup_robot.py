import RPi.GPIO as GPIO
import subprocess
import time

# Número del GPIO del final de carrera
PIN_FINAL_CARRERA = 17

def start():
    # Configuració del GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_FINAL_CARRERA, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    try:
        while True:
            # Si el final de carrera **NO està premut**
            if GPIO.input(PIN_FINAL_CARRERA) == GPIO.HIGH:
                print("FINAL DE CARRERA NO PREMUT - S’ACTIVA EL ROBOT")

                # Llança el launch file
                subprocess.Popen([
                    'ros2', 'launch', 'launch_file', 'launch.py'
                ])
                break

            # Esperar una mica per no saturar la CPU
            time.sleep(0.1)
    finally:
        GPIO.cleanup()