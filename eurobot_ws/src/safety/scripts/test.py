import RPi.GPIO as GPIO
import time

# Usar numeración BCM
GPIO.setmode(GPIO.BCM)

# Usar GPIO 21 (pin físico 40)
GPIO_PIN = 21

# Configurar como entrada con pull-down interno
GPIO.setup(GPIO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print(f"Leyendo GPIO {GPIO_PIN} (pin físico 40). Pulsa Ctrl+C para salir.")

try:
    while True:
        state = GPIO.input(GPIO_PIN)
        print(f"Estado: {'ALTO (1)' if state else 'BAJO (0)'}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nTerminando...")
finally:
    GPIO.cleanup()
