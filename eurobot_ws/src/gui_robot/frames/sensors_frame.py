import tkinter.font as tkFont
import os
import serial
import threading
from PIL import Image, ImageTk
from time import sleep

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None
sensor_states = [False, False, False, False]  # Sensor states (True = ON, False = OFF)

# Set up serial port
port = '/dev/ttyUSB0'  # Replace with your serial port
baudrate = 115200
serial_port = None

def open_serial_connection():
    """Open the serial port."""
    global serial_port
    try:
        serial_port = serial.Serial(port, baudrate, timeout=1)
        print(f"Serial port {port} opened successfully.")
    except serial.SerialException:
        print(f"Could not open serial port {port}.")
        serial_port = None

def read_from_serial():
    """Read incoming data from the serial port."""
    while True:
        if serial_port and serial_port.in_waiting > 0:
            message = serial_port.readline().decode('utf-8').strip()
            if message:
                print(f"Received: {message}")
                process_message(message)
        sleep(0.1)  # Small delay to avoid overwhelming the CPU

def process_message(message):
    """Process the message to update the sensor states and images."""
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
    """Update the image of the sensor based on its state."""
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

def sensors_frame(canvas):
    """ Set the function to design the frame00 (Control Systems Frame)
    
    Args:
        (canvas): Variable that set the shape of the window
    """
    global green_photo, red_photo, sensor_1_img, sensor_2_img, sensor_3_img, sensor_4_img  # Set the global variables
    from frames.main_frame import tk

    courirer_font = tkFont.Font(family="Courier", size=35) # Set the font for the default text
    esp_font = tkFont.Font(family="Courier", size=20) # Set the font for the ESP text

    # Set the variable that contains the path of the image
    green_path = os.path.join(current_directory, "../img/green.png") 
    green_image = Image.open(green_path)
    green_image = green_image.resize((22, 22), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((22, 22), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=28)
    font_3 = tkFont.Font(family="Courier", size=26)
    font_4 = tkFont.Font(family="Courier", size=10)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=96)

    canvas.create_text(750, 100, text="Sensors", font=font_title, fill="White", anchor="center")

    canvas.create_text(470, 180, text="Finals de Carrera", font=font_1, fill="White", anchor="w")
    canvas.create_text(470, 225, text="Final de Carrera 1", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 255, text="Final de Carrera 2", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 285, text="Final de Carrera 3", font=font_3, fill="White", anchor="w")
    canvas.create_text(470, 315, text="Final de Carrera 4", font=font_3, fill="White", anchor="w")

    sensor_1_img = canvas.create_image(440, 225, image=red_photo, anchor="w")
    sensor_2_img = canvas.create_image(440, 255, image=red_photo, anchor="w")
    sensor_3_img = canvas.create_image(440, 285, image=red_photo, anchor="w")
    sensor_4_img = canvas.create_image(440, 315, image=red_photo, anchor="w")

    # Start the serial reading thread
    threading.Thread(target=read_from_serial, daemon=True).start()

    open_serial_connection()
