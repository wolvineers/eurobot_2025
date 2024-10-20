import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")

#Set the function to print the motor veolocity in real time
def set_motor_velocity(value, motor):
    print(f"Velocidad del {motor}: {value}")

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
        command=lambda value: set_motor_velocity(value, "Motor 1")  
    )
    canvas.create_window(900, 165, window=slider1)

    slider2 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "Motor 2")  
    )
    canvas.create_window(900, 240, window=slider2)

    slider3 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "Motor 3")  
    )
    canvas.create_window(900, 320, window=slider3)

    slider4 = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_motor_velocity(value, "Motor 4")  
    )
    canvas.create_window(900, 400, window=slider4)
