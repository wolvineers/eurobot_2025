import tkinter.font as tkFont
import tkinter as tk
import os
import json
from PIL import Image, ImageTk
import os
import sys

# Get the absolute path of the project root (one level up from gui_zdc_base)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

# Now import from utils
from utils.src.serial_communication import open_serial_port, send_message

# Limpiar terminal (no necesario si no se está usando)
os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Path to the JSON files
modes_data_path = os.path.join("modes_data.json")
offset_data_path = os.path.join("servo_offsets.json")

# Function to load the mode data from the JSON file
def load_mode_data():
    try:
        with open(modes_data_path, "r") as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print("Error: modes_data.json not found.")
        return {}

# Function to load the servo offset data from the JSON file
def load_servo_offsets():
    try:
        with open(offset_data_path, "r") as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print("Error: servo_offsets.json not found.")
        return {}

# Function to get the current mode (1 or 2) from selector_data.txt
def get_current_mode():
    selector_file = os.path.join("selector_data.txt")
    try:
        with open(selector_file, "r") as file:
            mode = file.read().strip()
            return mode
    except FileNotFoundError:
        print("Error: selector_data.txt not found.")
        return "1"  # Default to mode 1 if the file doesn't exist

# Set the function to send the servo velocity to the ESP32
def set_servo_velocity(value, servos, mode_data, offset_data):
    port = '/dev/ttyUSB0'
    baudrate = 115200
    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return

    # Get the current mode (1 or 2)
    current_mode = get_current_mode()
    
    # Get the servo offsets for the current mode
    if current_mode in offset_data:
        offsets = offset_data[current_mode]
    else:
        print("Error: Mode not found in offset data.")
        return

    # Loop through servos and send values with offset
    for servo in servos:
        # Get the offset for the current servo
        offset = offsets.get(servo, 0)  # Default to 0 if the servo is not in the offset data

        # If the offset is negative, it means the servo is reversed, and the absolute value is the offset to add
        if offset < 0:
            adjusted_value = int(value) + abs(offset)  # Reverse direction and add the absolute offset
        else:
            adjusted_value = int(value) + offset  # Add the positive offset

        adjusted_value = max(0, min(360, adjusted_value))  # Ensure it's within bounds

        print(f"Servo {servo} - Sending value {adjusted_value}% (with offset {offset})")
        send_message(serial_port, f"{servo},{adjusted_value}")

# Function to update the slider value visually
servo_slider_values = {
    "S01": 0,
    "S02": 0,
    "S03": 0,
    "S04": 0,
    "S05": 0,
    "S06": 0,
    "S07": 0,
    "S08": 0
}

def change_slider_value(motor_id, increment):
    if motor_id in servo_slider_values:
        # Update the motor value
        new_value = servo_slider_values[motor_id] + increment
        # Ensure the value stays within the slider bounds (-100 to 100)
        servo_slider_values[motor_id] = max(0, min(360, new_value))
        
        # Update the slider widget visually
        slider = servo_sliders.get(motor_id)
        if slider:
            slider.set(servo_slider_values[motor_id])  # Update the slider visually

# Function to create a slider and store it in the servo_sliders mapping
def create_slider(canvas, x, y, servos_id, mode_data, offset_data):
    slider = tk.Scale(
        canvas,
        from_=0,
        to=360,
        orient=tk.HORIZONTAL,
        length=200,
        width=15,
        bd=0,
        command=lambda value: set_servo_velocity(value, servos_id, mode_data, offset_data),
    )
    canvas.create_window(x, y, window=slider, anchor="center")
    return slider

# Function to set the frame (Groups Frame)
def groups_frame(canvas):
    global button_photo, up_photo, down_photo, left_photo, right_photo

    # Load mode data (for groups) and servo offsets data (for servo adjustments)
    mode_data = load_mode_data()
    offset_data = load_servo_offsets()
    current_mode = get_current_mode()  # Get the current mode (1 or 2)
    
    # Get the groups for the current mode
    if current_mode in mode_data:
        groups = mode_data[current_mode]
    else:
        groups = {}

    # Images for arrows and buttons
    arrow_path = os.path.join(current_directory, "../img/icons8-arrow-96.png")
    arrow_image = Image.open(arrow_path)
    arrow_image = arrow_image.resize((75, 75), Image.LANCZOS)
    up_photo = ImageTk.PhotoImage(arrow_image)
    down_photo = ImageTk.PhotoImage(arrow_image.rotate(180))
    right_photo = ImageTk.PhotoImage(arrow_image.rotate(270))
    left_photo = ImageTk.PhotoImage(arrow_image.rotate(90))

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((250, 45), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=32)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    # Set the frame title
    canvas.create_text(750, 100, text="GROUPS", font=font_title, fill="White", anchor="center")

    # Display group titles and the motors for each group
    y_position = 200
    for group_name, motors in groups.items():
        canvas.create_text(450, y_position, text=group_name, font=font_1, fill="White", anchor="w")
        x_position = 450
        # List motors under the group, pass both mode_data and offset_data to the slider creation
        create_slider(canvas, 900, y_position, motors, mode_data, offset_data)
        # Add spacing between groups
        y_position += 180
