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

        # Add offset to the value
        adjusted_value = int(value) + abs(offset)
        adjusted_value = max(0, min(360, adjusted_value))  # Ensure it's within bounds

        # If the offset is -1, reverse the direction (180 - value)
        if offset < 0:
            adjusted_value = 180 - adjusted_value  # Reverse direction
        
        print(f"Servo {servo} - Sending value {adjusted_value}% (with offset {offset})")
        send_message(serial_port, f"{servo},{adjusted_value}")

# Create a dictionary to store the checkbox states (True for checked, False for unchecked)
# Initialize global angle variable
angle = 90  # Default angle
checkbox_states = {f"S0{i+1}": False for i in range(8)}

def update_angle(value, canvas, increment=False):
    global angle
    if increment:
        angle = max(0, min(360, angle+value))
    else: angle = max(0, min(360, value))  # Asegurar que el ángulo está en el rango permitido
    canvas.itemconfig(angle_label, text=f"Angle: {angle}º")  # ACTUALIZAR TEXTO CORRECTAMENTE


def send_angle():
    selected_servos = [servo for servo, state in checkbox_states.items() if state]
    if selected_servos:
        print("kjadsflksd")
        set_servo_velocity(angle, selected_servos, load_mode_data(), load_servo_offsets())

empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

empty_checkbox_image = Image.open(empty_checkbox_path)
selected_checkbox_image = Image.open(selected_checkbox_path)
empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)
# Function to toggle the checkbox state
# Function to toggle the checkbox state
def update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    existing_image = canvas.find_withtag(servo_id)
    if checkbox_states[servo_id]:
        image_to_display = checkbox_selected_photo
    else:
        image_to_display = checkbox_empty_photo

    # If the image already exists, just update it
    if existing_image:
        canvas.itemconfig(existing_image[0], image=image_to_display)
    else:
        canvas.create_image(x, y, image=image_to_display, anchor="center", tags=servo_id)

def toggle_checkbox(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    checkbox_states[servo_id] = not checkbox_states[servo_id]
    update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)
    print(checkbox_states)

def create_checkbox(canvas, x, y, servo_id):
    # Load images for this specific checkbox (instead of using global variables)
    empty_checkbox_image = Image.open(empty_checkbox_path)
    selected_checkbox_image = Image.open(selected_checkbox_path)
    empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
    selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)

    checkbox_empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    checkbox_selected_photo = ImageTk.PhotoImage(selected_checkbox_image)

    # Set initial checkbox image (unchecked state)
    update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)

    canvas.create_text(x+30, y, text=servo_id, font=tkFont.Font(family="Courier", size=32), fill="White", anchor="w")
    # Bind the click event to toggle the checkbox state
    canvas.tag_bind(servo_id, "<Button-1>", lambda e: toggle_checkbox(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo))

def create_angle_buttons(canvas):
    global angle_label, button_photo, button_photo_2
    font_btn = tkFont.Font(family="Courier", size=20)
    font_title = tkFont.Font(family="Courier", size=32)

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image_2 = button_image.resize((280, 50), Image.LANCZOS)
    button_image = button_image.resize((100, 50), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    button_photo_2 = ImageTk.PhotoImage(button_image_2)
    
    font_1 = tkFont.Font(family="Courier", size=36)
    
    # Display selected angle
    angle_label = canvas.create_text(800, 535, text=f"Angle: {angle}º", font=font_title, fill="white")

    # Botones de ángulos preestablecidos
    preset_angles = [45, 90, 135, 180]
    for i, preset in enumerate(preset_angles):
        y_pos = 175 + (i * 75)
        img_btn = canvas.create_image(700, y_pos, image=button_photo, anchor="center")
        txt_btn = canvas.create_text(700, y_pos, text=f"{preset}º", font=font_1, fill="white", anchor="center")
        
        # Vincular clics al botón
        canvas.tag_bind(img_btn, "<Button-1>", lambda e, p=preset: update_angle(p, canvas))
        canvas.tag_bind(txt_btn, "<Button-1>", lambda e, p=preset: update_angle(p, canvas))

    # Botones de incremento/decremento
    increments = [("+1", 1), ("-1", -1), ("+5", 5), ("-5", -5)]
    for i, (label, inc) in enumerate(increments):
        y_pos = 175 + (i * 75)
        img_btn = canvas.create_image(900, y_pos, image=button_photo, anchor="center")
        txt_btn = canvas.create_text(900, y_pos, text=label, font=font_1, fill="white", anchor="center")

        # Vincular clics al botón
        canvas.tag_bind(img_btn, "<Button-1>", lambda e, inc=inc: update_angle(inc, canvas, True))
        canvas.tag_bind(txt_btn, "<Button-1>", lambda e, inc=inc: update_angle(inc, canvas, True))


    # Crear botón como imagen y texto en el Canvas
    img_button = canvas.create_image(800, 485, image=button_photo_2, anchor="center")
    txt_button = canvas.create_text(800, 485, text="Send Angle", font=font_1, fill="white", anchor="center")

    # Asignar evento de clic
    canvas.tag_bind(img_button, "<Button-1>", lambda e: send_angle())
    canvas.tag_bind(txt_button, "<Button-1>", lambda e: send_angle())

# Function to set the frame (Groups Frame)
def groups_frame(canvas):
    global button_photo, up_photo, down_photo, left_photo, right_photo, back_photo, background_photo
    from frames.main_frame import switch_frame, window
    from frames.control_zone_frame import control_zone_frame

    # Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

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

    # Load mode data (for groups) and servo offsets data (for servo adjustments)
    offset_data = load_servo_offsets()
    current_mode = get_current_mode()  # Get the current mode (1 or 2)

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
    # Creating text and arrows
    font_title = tkFont.Font(family="Courier", size=64)
    canvas.create_text(600, 50, text="SERVOS", font=font_title, fill="White", anchor="center")

    # Create checkboxes for each servo using images
    create_checkbox(canvas, 250, 175, "S01")
    create_checkbox(canvas, 250, 250, "S02")
    create_checkbox(canvas, 250, 325, "S03")
    create_checkbox(canvas, 250, 400, "S04")
    create_checkbox(canvas, 400, 175, "S05")
    create_checkbox(canvas, 400, 250, "S06")
    create_checkbox(canvas, 400, 325, "S07")
    create_checkbox(canvas, 400, 400, "S08")

    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(control_zone_frame))

    create_angle_buttons(canvas)
