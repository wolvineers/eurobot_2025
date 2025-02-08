import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
#from utils.src.serial_communication import open_serial_port, send_message

os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set the function to send the motor velocity to the ESP32
def set_motor_velocity(value, motor):
    '''port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {motor}: {value}%")

    send_message(serial_port, motor+","+value)'''


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

# Function to update the slider value visually
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
def create_slider(canvas, x, y, motor_id, command_function):
    slider = tk.Scale(
        canvas,
        from_=-100,
        to=100,
        command=lambda value: command_function(value, motor_id),
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
    slider.tk.call(slider._w, 'configure', '-troughcolor', '#d7d7d7')
    slider.tk.call(slider._w, 'configure', '-sliderrelief', 'flat')

def motor_frame(canvas_ref):
    global canvas, green_photo, red_photo, left_photo, right_photo  # Set the global variables
    canvas = canvas_ref  # Assign the canvas to the global variable

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

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=24)
    font_2_orb = tkFont.Font(family="Orbitron", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    # Creating text and arrows
    canvas.create_text(750, 10, text="MOTORS", font=font_title, fill="White", anchor="center")

    # Motor 1
    canvas.create_text(600, 180, text="MOTOR 1", font=font_2, fill="White", anchor="center")
    arrowleft1 = canvas.create_image(475, 215, image=left_photo, anchor="center")
    arrowright1 = canvas.create_image(725, 215, image=right_photo, anchor="center")

    # Motor 2
    canvas.create_text(600, 255, text="MOTOR 2", font=font_2, fill="White", anchor="center")
    arrowleft2 = canvas.create_image(475, 290, image=left_photo, anchor="center")
    arrowright2 = canvas.create_image(725, 290, image=right_photo, anchor="center")

    # Motor 3
    canvas.create_text(600, 330, text="MOTOR 3", font=font_2, fill="White", anchor="center")
    arrowleft3 = canvas.create_image(475, 365, image=left_photo, anchor="center")
    arrowright3 = canvas.create_image(725, 365, image=right_photo, anchor="center")

    # Motor 4
    canvas.create_text(600, 405, text="MOTOR 4", font=font_2, fill="White", anchor="center")
    arrowleft4 = canvas.create_image(475, 440, image=left_photo, anchor="center")
    arrowright4 = canvas.create_image(725, 440, image=right_photo, anchor="center")

    # Create sliders using the helper function
    create_slider(canvas, 600, 215, "M01", change_slider_value)
    create_slider(canvas, 600, 290, "M02", change_slider_value)
    create_slider(canvas, 600, 365, "M03", change_slider_value)
    create_slider(canvas, 600, 440, "M04", change_slider_value)

    # Binding button presses for continuous action for each motor
    canvas.tag_bind(arrowleft1, "<ButtonPress-1>", lambda e: on_button_press("M01", -1))
    canvas.tag_bind(arrowright1, "<ButtonPress-1>", lambda e: on_button_press("M01", 1))
    canvas.tag_bind(arrowleft2, "<ButtonPress-1>", lambda e: on_button_press("M02", -1))
    canvas.tag_bind(arrowright2, "<ButtonPress-1>", lambda e: on_button_press("M02", 1))
    canvas.tag_bind(arrowleft3, "<ButtonPress-1>", lambda e: on_button_press("M03", -1))
    canvas.tag_bind(arrowright3, "<ButtonPress-1>", lambda e: on_button_press("M03", 1))
    canvas.tag_bind(arrowleft4, "<ButtonPress-1>", lambda e: on_button_press("M04", -1))
    canvas.tag_bind(arrowright4, "<ButtonPress-1>", lambda e: on_button_press("M04", 1))

    # Stop on release for each motor
    canvas.tag_bind(arrowleft1, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright1, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft2, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright2, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft3, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright3, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft4, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright4, "<ButtonRelease-1>", on_button_release)

    canvas.create_text(900, 220, text="FdC 1", font=font_2, fill="White", anchor="w")
    canvas.create_text(900, 260, text="FdC 2", font=font_2, fill="White", anchor="w")
    canvas.create_text(900, 300, text="FdC 3", font=font_2, fill="White", anchor="w")
    canvas.create_text(900, 340, text="FdC 4", font=font_2, fill="White", anchor="w")
    canvas.create_text(900, 380, text="Encoder 1", font=font_2, fill="White", anchor="w")
    canvas.create_text(900, 420, text="Encoder 2", font=font_2, fill="White", anchor="w")

    canvas.create_image(875, 220, image=green_photo, anchor="e")
    canvas.create_image(875, 260, image=red_photo, anchor="e")
    canvas.create_image(875, 300, image=red_photo, anchor="e")
    canvas.create_image(875, 340, image=red_photo, anchor="e")
    canvas.create_text(875, 380, text="(16)", font=font_2_orb, fill="White", anchor="e")
    canvas.create_text(875, 420, text="(24)", font=font_2_orb, fill="White", anchor="e")