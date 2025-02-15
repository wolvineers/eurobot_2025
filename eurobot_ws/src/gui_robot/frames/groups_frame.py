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

# Set the function to send the servo velocity to the ESP32 (comentada por ahora)
def set_servo_velocity(value, servos):
    port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 
    for servo in servos:
        print(f"Position {servo}: {value}%")
        send_message(serial_port, servo+","+value)

green_photo = None
red_photo = None
green_square_photo = None

# Path to the JSON file containing motor group data
modes_data_path = os.path.join("modes_data.json")

# Function to load the mode data from the JSON file
def load_mode_data():
    try:
        with open(modes_data_path, "r") as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print("Error: modes_data.json not found.")
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

# Function to update the slider value visually
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
def create_slider(canvas, x, y, servos_id):
    slider = tk.Scale(
        canvas,
        from_=0,
        to=360,
        orient=tk.HORIZONTAL,
        length=200,
        width=15,
        bd=0,
        command=lambda value: set_servo_velocity(value, servos_id),
    )
    canvas.create_window(x, y, window=slider, anchor="center")
    return slider

def groups_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo, up_photo, down_photo, left_photo, right_photo  # Set the global variables

    # Load motor group data
    mode_data = load_mode_data()
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
        # List motors under the group
        create_slider(canvas, 900, y_position, motors)
        # Add spacing between groups
        y_position += 180

'''
# Create arrows and buttons as previously defined
        canvas.create_image(850, y_position - 40, image=up_photo, anchor="center")
        canvas.create_image(950, y_position - 40, image=down_photo, anchor="center")
        canvas.create_image(850, y_position + 60, image=left_photo, anchor="center")
        canvas.create_image(950, y_position + 60, image=right_photo, anchor="center")
        canvas.create_image(900, y_position + 120, image=button_photo, anchor="center")
        canvas.create_text(900, y_position + 120, text="MOVE", font=font_2, fill="White", anchor="center")
'''