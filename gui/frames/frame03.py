import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk


def frame03(canvas):
    global servo_photo, camera_photo, motor_photo, camera_button

    # Cargar y redimensionar la imagen
    servo_image = Image.open("gui/img/servo_white.png")
    servo_image = servo_image.resize((150, 150), Image.LANCZOS)
    servo_photo = ImageTk.PhotoImage(servo_image)

    camera_image = Image.open("gui/img/camera_white.png")
    camera_image = camera_image.resize((150, 150), Image.LANCZOS)
    camera_photo = ImageTk.PhotoImage(camera_image)

    motor_image = Image.open("gui/img/motor_white.png")
    motor_image = motor_image.resize((150, 150), Image.LANCZOS)
    motor_photo = ImageTk.PhotoImage(motor_image)

    # Añadir la imagen al canvas
    servo_button = canvas.create_image(560, 100, image=servo_photo, anchor="nw")  
    camera_button = canvas.create_image(750, 300, image=camera_photo, anchor="nw")
    motor_button = canvas.create_image(450, 300, image=motor_photo, anchor="nw")  

    canvas.tag_bind(servo_button, "<Button-1>", lambda e: print("Hola"))
    canvas.tag_bind(camera_button, "<Button-1>", lambda e: print("Hola"))
    canvas.tag_bind(motor_button, "<Button-1>", lambda e: print("Hola"))
    




