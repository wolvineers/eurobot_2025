import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
#from utils.src.serial_communication import open_serial_port, send_message

os.system("clear")

#Set the function to send the motor veolocity to the ESP32
def set_motor_velocity(value, motor):
    '''port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {motor}: {value}%")

    send_message(serial_port, motor+","+value)'''

#Set the function to design the motor_frame

def create_slider(canvas, x, y, motor_id, command_function):
    '''
    Create and customize a slider, then place it on the canvas.

    Args:
        canvas: The canvas where the slider will be placed.
        x: The x-coordinate for the slider's position.
        y: The y-coordinate for the slider's position.
        motor_id: The ID of the motor (e.g., "M01", "M02").
        command_function: The function to call when the slider value changes.
    '''
    slider = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        command=lambda value: command_function(value, motor_id),
    )
    customize_slider(slider)
    canvas.create_window(x, y, window=slider, anchor="center")

# Customize the slider for a better look
def customize_slider(slider):
    """
    Customize the appearance of the slider.

    Args:
        slider: The slider widget to customize.
    """
    slider.config(
        orient=tk.HORIZONTAL, 
        length=200,
        width=15,
        bd=0,
    )
    slider.tk.call(slider._w, 'configure', '-troughcolor', '#d7d7d7')
    slider.tk.call(slider._w, 'configure', '-sliderrelief', 'flat')


def motor_frame(canvas):
    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=24)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="MOTORS", font=font_title, fill="White", anchor="center")

    canvas.create_text(600, 200, text="MOTOR 1", font=font_2, fill="White", anchor="center")
    canvas.create_text(600, 300, text="MOTOR 2", font=font_2, fill="White", anchor="center")
    canvas.create_text(600, 400, text="MOTOR 3", font=font_2, fill="White", anchor="center")
    canvas.create_text(950, 200, text="MOTOR 4", font=font_2, fill="White", anchor="center")
    canvas.create_text(950, 300, text="MOTOR 5", font=font_2, fill="White", anchor="center")
    canvas.create_text(950, 400, text="MOTOR 6", font=font_2, fill="White", anchor="center")

    # Create sliders using the helper function
    create_slider(canvas, 600, 235, "M01", set_motor_velocity)
    create_slider(canvas, 600, 335, "M02", set_motor_velocity)
    create_slider(canvas, 600, 435, "M03", set_motor_velocity)
    create_slider(canvas, 950, 235, "M04", set_motor_velocity)
    create_slider(canvas, 950, 335, "M05", set_motor_velocity)
    create_slider(canvas, 950, 435, "M06", set_motor_velocity)