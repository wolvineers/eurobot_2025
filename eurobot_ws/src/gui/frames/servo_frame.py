import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")

#Set the function to print the servo angle in real time
def set_servo_position(value, servo):
    print(f"Posición del {servo}: {value}°")

#Set the function to design the servo_frame
def servo_frame(canvas):
    courirer_font = tkFont.Font(family="Courier", size=35)

    canvas.create_text(550, 170, text="SERVO 1", font=courirer_font, fill="White")
    canvas.create_text(550, 250, text="SERVO 2", font=courirer_font, fill="White")
    canvas.create_text(550, 330, text="SERVO 3", font=courirer_font, fill="White")
    canvas.create_text(550, 410, text="SERVO 4", font=courirer_font, fill="White")


    slider1 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_servo_position(value, "Servo 1")  
    )
    canvas.create_window(900, 165, window=slider1)

    slider2 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_servo_position(value, "Servo 2")  
    )
    canvas.create_window(900, 240, window=slider2)

    slider3 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_servo_position(value, "Servo 3")  
    )
    canvas.create_window(900, 320, window=slider3)

    slider4 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        orient=tk.HORIZONTAL,
        length=300,
        command=lambda value: set_servo_position(value, "Servo 4")  
    )
    canvas.create_window(900, 400, window=slider4)
