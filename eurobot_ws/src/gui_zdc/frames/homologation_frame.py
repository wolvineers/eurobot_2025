import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk
from frames.welcome_frame import welcome_frame
import sys

# Get the absolute path of the project root (one level up from gui_zdc_base)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

# Now import from utils
from utils.src.serial_communication import open_serial_port, send_message

# Limpiar terminal (no necesario si no se está usando)
os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set the function to send the servo velocity to the ESP32 (comentada por ahora)
def set_servo_velocity(value, servo):
    port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {servo}: {value}%")

    send_message(serial_port, servo+","+value)


def open_robot(canvas):
    set_servo_velocity(0,"S01")
    set_servo_velocity(0,"S02")
    set_servo_velocity(0,"S03")
    set_servo_velocity(0,"S04")

def close_robot(canvas):
    set_servo_velocity(0,"S01")
    set_servo_velocity(0,"S02")
    set_servo_velocity(0,"S03")
    set_servo_velocity(0,"S04")


# Initialize the frame
def homologation_frame(canvas):
    global button_photo, back_photo, background_photo
    canvas.image_cache = {}

    from frames.main_frame import switch_frame, window

    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    font_title = tkFont.Font(family="Courier", size=54)
    font_2 = tkFont.Font(family="Courier", size=38)

    back_path = os.path.join(current_directory, "../img/icons8-back-24.png")
    back_image = Image.open(back_path)
    back_image = back_image.resize((48, 48), Image.LANCZOS)
    back_photo = ImageTk.PhotoImage(back_image)


    background_path = os.path.join(current_directory, "../img/background.jpg")
    background_image = Image.open(background_path)
    background_image = background_image.resize((int(screen_width), int(screen_height)), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)

    canvas.create_image(0, 0, image=background_photo, anchor="nw")
    img_back = canvas.create_image(24, 24, image=back_photo, anchor="nw")
    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(welcome_frame))

    canvas.create_text(600, 125, text="HOMOLOGATION", font=font_title, fill="White", anchor="center")

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((600, 75), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    img_open = canvas.create_image(600, 350, image=button_photo, anchor="center")
    txt_open = canvas.create_text(600, 350, text="OPEN", font=font_2, fill="White", anchor="center")

    canvas.tag_bind(img_open, "<Button-1>", lambda e: open_robot(canvas))
    canvas.tag_bind(txt_open, "<Button-1>", lambda e: open_robot(canvas))

    img_close = canvas.create_image(600, 450, image=button_photo, anchor="center")
    txt_close = canvas.create_text(600, 450, text="CLOSE", font=font_2, fill="White", anchor="center")

    canvas.tag_bind(img_close, "<Button-1>", lambda e: close_robot(canvas))
    canvas.tag_bind(txt_close, "<Button-1>", lambda e: close_robot(canvas))


