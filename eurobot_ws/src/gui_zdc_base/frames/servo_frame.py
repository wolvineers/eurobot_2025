import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
#from utils.src.serial_communication import open_serial_port, send_message

import serial_communication

#from serial_communication import open_serial_port, send_message

os.system("clear")

<<<<<<< HEAD
=======


>>>>>>> development
#Set the function to send the servo angle to the ESP32
def set_servo_position(value, servo):
    '''port = '/dev/ttyUSB0'
    baudrate = 115200

<<<<<<< HEAD
    serial_port = open_serial_port(port, baudrate)
=======
    serial_port = serial_communication.open_serial_port(port, baudrate)
>>>>>>> development

    if not serial_port:
        print("Could not open serial port.")
        return

    print(f"Position {servo}: {value}°")

<<<<<<< HEAD:eurobot_ws/src/gui/frames/servo_frame.py
<<<<<<< HEAD
    send_message(serial_port, servo+","+value)
=======
    serial_communication.send_message(serial_port, servo+","+value)
>>>>>>> development

=======
    send_message(serial_port, servo+","+value)'''
>>>>>>> eur-35-configure-actuators:eurobot_ws/src/gui_zdc_base/frames/servo_frame.py

#Set the function to design the servo_frame
def servo_frame(canvas):



    courirer_font = tkFont.Font(family="Courier", size=35)

    canvas.create_text(550, 170, text="SERVO 1", font=courirer_font, fill="White")
    canvas.create_text(550, 250, text="SERVO 2", font=courirer_font, fill="White")
    canvas.create_text(550, 330, text="SERVO 3", font=courirer_font, fill="White")
    canvas.create_text(550, 410, text="SERVO 4", font=courirer_font, fill="White")

    #Customize the slider for a better look
    def customize_slider(slider):
        slider.config(
            orient=tk.HORIZONTAL, 
            length=300,
            width=20,
            bd=0,
        )
        
        slider.tk.call(slider._w, 'configure', '-troughcolor', '#d7d7d7')
        slider.tk.call(slider._w, 'configure', '-sliderrelief', 'flat')


    slider1 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        command=lambda value: set_servo_position(value, "S01")  
    )
    customize_slider(slider1)
    canvas.create_window(850, 170, window=slider1)

    slider2 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        command=lambda value: set_servo_position(value, "S02")  
    )
    customize_slider(slider2)
    canvas.create_window(850, 250, window=slider2)

    slider3 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        command=lambda value: set_servo_position(value, "S03")  
    )
    customize_slider(slider3)
    canvas.create_window(850, 330, window=slider3)

    slider4 = tk.Scale(
        canvas,
        from_=0,
        to=180,
        command=lambda value: set_servo_position(value, "S04")  
    )
    customize_slider(slider4)
    canvas.create_window(850, 410, window=slider4)
