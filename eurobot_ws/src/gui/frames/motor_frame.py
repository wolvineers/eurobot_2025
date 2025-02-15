import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
from utils.src.serial_communication import open_serial_port, send_message

os.system("clear")

#Set the function to send the motor veolocity to the ESP32
def set_motor_velocity(value, motor):

    port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {motor}: {value}%")

    send_message(serial_port, motor+","+value)

#Set the function to design the motor_frame
def motor_frame(canvas):
    courirer_font = tkFont.Font(family="Courier", size=35)

    canvas.create_text(550, 170, text="MOTOR 1", font=courirer_font, fill="White")
    canvas.create_text(550, 250, text="MOTOR 2", font=courirer_font, fill="White")
    canvas.create_text(550, 330, text="MOTOR 3", font=courirer_font, fill="White")
    canvas.create_text(550, 410, text="MOTOR 4", font=courirer_font, fill="White")


    slider1 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "M01")  
    )
    canvas.create_window(900, 165, window=slider1)

    slider2 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "M02")  
    )
    canvas.create_window(900, 240, window=slider2)

    slider3 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "M03")  
    )
    canvas.create_window(900, 320, window=slider3)

    slider4 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "M04")  
    )
    canvas.create_window(900, 400, window=slider4)
