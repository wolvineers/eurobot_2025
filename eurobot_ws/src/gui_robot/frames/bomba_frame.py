import tkinter as tk
import tkinter.font as tkFont
import os
import serial
import threading
from PIL import Image, ImageTk
from time import sleep

# Set the absolute path
current_directory = os.path.dirname(os.path.abspath(__file__))

# Global variables
green_photo = None
red_photo = None
sensor_states = [False, False, False, False]  # Sensor states (True = ON, False = OFF)
checkbox_states = {f"AB0{i+1}": False for i in range(2)}  # Initialize checkbox states

# Serial port settings
port = '/dev/ttyUSB0'  # Replace with your serial port
baudrate = 115200
serial_port = None

# Define serial communication functions
def open_serial_connection():
    global serial_port
    try:
        serial_port = serial.Serial(port, baudrate, timeout=1)
        print(f"Serial port {port} opened successfully.")
    except serial.SerialException:
        print(f"Could not open serial port {port}.")
        serial_port = None

def read_from_serial():
    while True:
        if serial_port and serial_port.in_waiting > 0:
            message = serial_port.readline().decode('utf-8').strip()
            if message:
                print(f"Received: {message}")
                process_message(message)
        sleep(0.1)

def process_message(message):
    global sensor_states
    if message.startswith("LS0"):
        parts = message.split(",")
        if len(parts) == 2:
            sensor_id = int(parts[0].replace("LS0", ""))
            state = int(parts[1])
            if 0 <= sensor_id < len(sensor_states):
                sensor_states[sensor_id] = (state == 1)
                update_sensor_image(sensor_id)

def update_sensor_image(sensor_id):
    global canvas, green_photo, red_photo
    if sensor_id == 0:
        if sensor_states[0]:
            canvas.itemconfig(sensor_1_img, image=green_photo)
        else:
            canvas.itemconfig(sensor_1_img, image=red_photo)
    elif sensor_id == 1:
        if sensor_states[1]:
            canvas.itemconfig(sensor_2_img, image=green_photo)
        else:
            canvas.itemconfig(sensor_2_img, image=red_photo)
    elif sensor_id == 2:
        if sensor_states[2]:
            canvas.itemconfig(sensor_3_img, image=green_photo)
        else:
            canvas.itemconfig(sensor_3_img, image=red_photo)
    elif sensor_id == 3:
        if sensor_states[3]:
            canvas.itemconfig(sensor_4_img, image=green_photo)
        else:
            canvas.itemconfig(sensor_4_img, image=red_photo)

# Checkbox functions
def update_checkbox_image(motor_id, x, y, canvas):
    existing_image = canvas.find_withtag(motor_id)
    if checkbox_states[motor_id]:
        image_to_display = checkbox_selected_photo
    else:
        image_to_display = checkbox_empty_photo

    if existing_image:
        canvas.itemconfig(existing_image[0], image=image_to_display)
    else:
        canvas.create_image(x, y, image=image_to_display, anchor="center", tags=motor_id)

def toggle_checkbox(motor_id, x, y, canvas):
    checkbox_states[motor_id] = not checkbox_states[motor_id]
    update_checkbox_image(motor_id, x, y, canvas)
    set_bomb_state(checkbox_states[motor_id], motor_id)

def create_checkbox(canvas, x, y, motor_id):
    update_checkbox_image(motor_id, x, y, canvas)
    canvas.create_text(x + 30, y + 5, text=motor_id, font=tkFont.Font(family="Courier", size=38), fill="White", anchor="w")
    canvas.tag_bind(motor_id, "<Button-1>", lambda e: toggle_checkbox(motor_id, x, y, canvas))

def set_bomb_state(value, bombs):
    if not serial_port:
        print("Could not open serial port.")
        return
    for bomb in bombs:
        print(f"AirBomb {bomb} - Sending value {value}%")
        send_message(serial_port, f"{bomb},{value}")

def send_message(serial_port, message):
    if serial_port:
        serial_port.write(message.encode('utf-8'))

# Create the combined frame
def airbomb_frame(canvas):
    global red_photo, green_photo, checkbox_empty_photo, checkbox_selected_photo
    font_title = tkFont.Font(family="Courier", size=64)
    font_1 = tkFont.Font(family="Courier", size=36)
    font_3 = tkFont.Font(family="Courier", size=26)


    # Load images for sensor states
    green_path = os.path.join(current_directory, "../img/green.png")
    green_image = Image.open(green_path)
    green_image = green_image.resize((22, 22), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((22, 22), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)

    # Load checkbox images
    empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
    selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

    empty_checkbox_image = Image.open(empty_checkbox_path)
    selected_checkbox_image = Image.open(selected_checkbox_path)
    empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
    selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)
    checkbox_empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    checkbox_selected_photo = ImageTk.PhotoImage(selected_checkbox_image)

    # Sensor titles and images
    canvas.create_text(750, 100, text="Sensors", font=font_title, fill="White", anchor="center")
    canvas.create_text(470, 180, text="Finals de Carrera", font=font_1, fill="White", anchor="w")
    canvas.create_text(470, 225, text="Final de Carrera 1", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 255, text="Final de Carrera 2", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 285, text="Final de Carrera 3", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 315, text="Final de Carrera 4", font=font_3, fill="White", anchor="w")

    global sensor_1_img, sensor_2_img, sensor_3_img, sensor_4_img
    sensor_1_img = canvas.create_image(440, 225, image=red_photo, anchor="w")
    sensor_2_img = canvas.create_image(440, 255, image=red_photo, anchor="w")
    sensor_3_img = canvas.create_image(440, 285, image=red_photo, anchor="w")
    sensor_4_img = canvas.create_image(440, 315, image=red_photo, anchor="w")

    # Create checkboxes for bombs
    canvas.create_text(470, 400, text="Air Bombs", font=font_1, fill="White", anchor="w")
    create_checkbox(canvas, 500, 445, "AB01")
    create_checkbox(canvas, 500, 500, "AB02")

    # Start the serial thread
    threading.Thread(target=read_from_serial, daemon=True).start()

    open_serial_connection()

