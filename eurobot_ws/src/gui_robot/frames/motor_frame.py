'''import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
import sys

# Get the absolute path of the project root (one level up from gui_zdc_base)
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

# Now import from utils
from utils.src.serial_communication import open_serial_port, send_message

os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set the function to send the motor velocity to the ESP32
def set_motor_velocity(value, motor):
    port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {motor}: {value}%")

    send_message(serial_port, motor+","+value)


# Global variables to store slider values and slider objects
motor_slider_values = {
    "M01": 0,
    "M02": 0,
    "M03": 0,
    "M04": 0
}

motor_sliders = {
    "M01": None,
    "M02": None,
    "M03": None,
    "M04": None
}

# Function to update the slider value visually and send command
def change_slider_value(motor_id, increment):
    if motor_id in motor_slider_values:
        # Update the motor value
        new_value = motor_slider_values[motor_id] + increment
        # Ensure the value stays within the slider bounds (-100 to 100)
        motor_slider_values[motor_id] = max(-100, min(100, new_value))
        
        # Update the slider widget visually
        slider = motor_sliders.get(motor_id)
        if slider:
            slider.set(motor_slider_values[motor_id])  # Update the slider visually
            
        # Send updated value to the motor
        set_motor_velocity(str(motor_slider_values[motor_id]), motor_id)

# Function to handle button press for continuous update
def on_button_press(motor_id, increment):
    global current_action, canvas  # Use the global canvas variable
    change_slider_value(motor_id, increment)
    # Continuously call the function every 100ms while button is pressed
    current_action = canvas.after(100, on_button_press, motor_id, increment)

# Function to stop the continuous action when button is released
def on_button_release(event=None):
    global current_action, canvas  # Use the global canvas variable
    if current_action:
        canvas.after_cancel(current_action)
        current_action = None

# Function to create a slider and store it in the motor_sliders mapping
def create_slider(canvas, x, y, motor_id):
    slider = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        command=lambda value: set_motor_velocity(value, motor_id),  # Directly call set_motor_velocity
    )
    customize_slider(slider)
    canvas.create_window(x, y, window=slider, anchor="center")
    motor_sliders[motor_id] = slider  # Store the slider in the global mapping
    return slider

# Customize the slider appearance
def customize_slider(slider):
    slider.config(
        orient=tk.HORIZONTAL, 
        length=200,
        width=15,
        bd=0,
    )

def motor_frame(canvas_ref):
    global canvas, green_photo, red_photo, left_photo, right_photo, background_photo, back_photo  # Set the global variables
    canvas = canvas_ref  # Assign the canvas to the global variable

    from frames.main_frame import switch_frame, window
    from frames.control_zone_frame import control_zone_frame
    
    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()


    arrow_path = os.path.join(current_directory, "../img/icons8-arrow-96.png")
    arrow_image = Image.open(arrow_path)
    arrow_image = arrow_image.resize((24, 24), Image.LANCZOS)
    up_photo = ImageTk.PhotoImage(arrow_image)
    down_photo = ImageTk.PhotoImage(arrow_image.rotate(180))
    right_photo = ImageTk.PhotoImage(arrow_image.rotate(270))
    left_photo = ImageTk.PhotoImage(arrow_image.rotate(90))

    # Define paths for motor status icons
    green_path = os.path.join(current_directory, "../img/green.png")
    green_image = Image.open(green_path)
    green_image = green_image.resize((24, 24), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((24, 24), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)


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

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=24)
    font_2_orb = tkFont.Font(family="Orbitron", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    # Creating text and arrows
    canvas.create_text(600, 100, text="MOTORS", font=font_title, fill="White", anchor="center")

    # Motor 1
    canvas.create_text(450, 180, text="MOTOR 1", font=font_2, fill="White", anchor="center")

    # Motor 2
    canvas.create_text(450, 255, text="MOTOR 2", font=font_2, fill="White", anchor="center")

    # Motor 3
    canvas.create_text(450, 330, text="MOTOR 3", font=font_2, fill="White", anchor="center")

    # Motor 4
    canvas.create_text(450, 405, text="MOTOR 4", font=font_2, fill="White", anchor="center")

    # Create sliders using the helper function
    create_slider(canvas, 450, 215, "M01")
    create_slider(canvas, 450, 290, "M02")
    create_slider(canvas, 450, 365, "M03")
    create_slider(canvas, 450, 440, "M04")

    canvas.create_text(750, 220, text="FdC 1", font=font_2, fill="White", anchor="w")
    canvas.create_text(750, 260, text="FdC 2", font=font_2, fill="White", anchor="w")
    canvas.create_text(750, 300, text="FdC 3", font=font_2, fill="White", anchor="w")
    canvas.create_text(750, 340, text="FdC 4", font=font_2, fill="White", anchor="w")
    canvas.create_text(750, 380, text="Encoder 1", font=font_2, fill="White", anchor="w")
    canvas.create_text(750, 420, text="Encoder 2", font=font_2, fill="White", anchor="w")

    canvas.create_image(725, 220, image=green_photo, anchor="e")
    canvas.create_image(725, 260, image=red_photo, anchor="e")
    canvas.create_image(725, 300, image=red_photo, anchor="e")
    canvas.create_image(725, 340, image=red_photo, anchor="e")
    canvas.create_text(725, 380, text="(16)", font=font_2_orb, fill="White", anchor="e")
    canvas.create_text(725, 420, text="(24)", font=font_2_orb, fill="White", anchor="e")

    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(control_zone_frame))









'''
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

# Set the function to send the servo velocity to the ESP32
def set_servo_velocity(value, servos):
    port = '/dev/ttyUSB0'
    baudrate = 115200
    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return
    
    # Loop through servos and send values with offset
    for servo in servos:
        # If the offset is -1, reverse the direction (180 - value)
        print(f"Motor {servo} - Sending value {value}%")
        send_message(serial_port, f"{servo},{value}")

# Create a dictionary to store the checkbox states (True for checked, False for unchecked)
# Initialize global angle variable
checkbox_states = {f"M0{i+1}": False for i in range(5)}

def send_angle(velocity):
    selected_motors = [motor for motor, state in checkbox_states.items() if state]
    if selected_motors:
        set_servo_velocity(velocity, selected_motors)

empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

empty_checkbox_image = Image.open(empty_checkbox_path)
selected_checkbox_image = Image.open(selected_checkbox_path)
empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)
# Function to toggle the checkbox state
# Function to toggle the checkbox state
def update_checkbox_image(motor_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    existing_image = canvas.find_withtag(motor_id)
    if checkbox_states[motor_id]:
        image_to_display = checkbox_selected_photo
    else:
        image_to_display = checkbox_empty_photo

    # If the image already exists, just update it
    if existing_image:
        canvas.itemconfig(existing_image[0], image=image_to_display)
    else:
        canvas.create_image(x, y, image=image_to_display, anchor="center", tags=motor_id)

def toggle_checkbox(motor_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    checkbox_states[motor_id] = not checkbox_states[motor_id]
    update_checkbox_image(motor_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)
    print(checkbox_states)

def create_checkbox(canvas, x, y, motor_id):
    # Load images for this specific checkbox (instead of using global variables)
    empty_checkbox_image = Image.open(empty_checkbox_path)
    selected_checkbox_image = Image.open(selected_checkbox_path)
    empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
    selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)

    checkbox_empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    checkbox_selected_photo = ImageTk.PhotoImage(selected_checkbox_image)

    # Set initial checkbox image (unchecked state)
    update_checkbox_image(motor_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)

    canvas.create_text(x+30, y+5, text=motor_id, font=tkFont.Font(family="Courier", size=32), fill="White", anchor="w")
    # Bind the click event to toggle the checkbox state
    canvas.tag_bind(motor_id, "<Button-1>", lambda e: toggle_checkbox(motor_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo))

def create_servo_buttons(canvas):
    global angle_label, button_photo, button_photo_2
    font_btn = tkFont.Font(family="Courier", size=20)
    font_title = tkFont.Font(family="Courier", size=32)

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image_2 = button_image.resize((350, 65), Image.LANCZOS)
    button_image = button_image.resize((100, 50), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    button_photo_2 = ImageTk.PhotoImage(button_image_2)
    
    font_1 = tkFont.Font(family="Courier", size=36)
    
    img_btn = canvas.create_image(700, 400, image=button_photo, anchor="center")
    txt_btn = canvas.create_text(700, 400, text=f"STOP", font=font_1, fill="white", anchor="center")
    
    # Vincular clics al botón
    canvas.tag_bind(img_btn, "<Button-1>", lambda e: send_angle(0))
    canvas.tag_bind(txt_btn, "<Button-1>", lambda e: send_angle(0))

    slider = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        command=lambda value: send_angle(value),  # Directly call set_motor_velocity
    )
    slider.config(
        orient=tk.VERTICAL, 
        length=200,
        width=40,
        bd=0,
    )
    canvas.create_window(800, 300, window=slider, anchor="center")

# Function to set the frame (Groups Frame)
def motor_frame(canvas):
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
    canvas.create_text(600, 100, text="MOTORS", font=font_title, fill="White", anchor="center")

    # Create checkboxes for each servo using images
    create_checkbox(canvas, 250, 175, "M01")
    create_checkbox(canvas, 250, 250, "M02")
    create_checkbox(canvas, 250, 325, "M03")
    create_checkbox(canvas, 250, 400, "M04")
    create_checkbox(canvas, 250, 400, "M05")

    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(control_zone_frame))

    create_servo_buttons(canvas)
