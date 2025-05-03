import RPi.GPIO as GPIO
import time
import subprocess

BUTTON_PIN = 8
GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


try:
    while True:
        button_state = GPIO.input(BUTTON_PIN)
        if button_state == GPIO.HIGH:
            print("START!")
            try:
                subprocess.run(['ros2', 'launch', 'launch_file', 'launch.py'], check=True)
            except subprocess.CalledProcessError as e:
                print(f"Error en executar el llançament: {e}")
            except Exception as e:
                print(f"Ha ocurregut un error inesperat: {e}")
        elif button_state == GPIO.LOW:
            print("WAIT")
        time.sleep(0.1)

except Exception as e:
    print(f"ERROR {e}")

finally:
    GPIO.cleanup()
