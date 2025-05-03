import RPi.GPIO as GPIO
import time
import subprocess

BUTTON_PIN = 8
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    lidar_process = subprocess.Popen(['ros2', 'launch', 'launch_file', 'launch.py'])
except Exception as e:
    print(f"Error lanzando launch.py: {e}")
    lidar_process = None

try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)
        if button_state == GPIO.HIGH:
            print("START!")
            try:
                subprocess.Popen(['ros2', 'launch', 'launch_file', 'start_launch.py'])
            except subprocess.CalledProcessError as e:
                print(f"Error en ejecutar el lanzamiento: {e}")
            except Exception as e:
                print(f"Ha ocurrido un error inesperado: {e}")
            break  
        else:
            print("WAIT")
        time.sleep(0.1)

except Exception as e:
    print(f"ERROR {e}")

finally:
    GPIO.cleanup()
