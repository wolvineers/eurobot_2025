import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

# Limpiar terminal (no necesario si no se está usando)
os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set the function to send the servo velocity to the ESP32 (comentada por ahora)
def set_servo_velocity(value, servo):
    '''port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {servo}: {value}%")

    send_message(serial_port, servo+","+value)'''


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

servo_sliders = {
    "S01": None,
    "S02": None,
    "S03": None,
    "S04": None,
    "S05": None,
    "S06": None,
    "S07": None,
    "S08": None
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
def create_slider(canvas, x, y, servo_id, command_function):
    slider = tk.Scale(
        canvas,
        from_=0,
        to=360,
        orient=tk.HORIZONTAL,
        length=200,
        width=15,
        bd=0,
        command=lambda value: command_function(value, servo_id),
    )
    slider.config(
        troughcolor='#d7d7d7', 
        sliderrelief='flat',
    )
    canvas.create_window(x, y, window=slider, anchor="center")
    servo_sliders[servo_id] = slider  # Store the slider in the global mapping
    return slider

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

def servo_frame(canvas_ref):
    global canvas, green_photo, red_photo, left_photo, right_photo  # Set the global variables
    canvas = canvas_ref  # Assign the canvas to the global variable

    arrow_path = os.path.join(current_directory, "../img/icons8-arrow-96.png")
    arrow_image = Image.open(arrow_path)
    arrow_image = arrow_image.resize((24, 24), Image.LANCZOS)
    up_photo = ImageTk.PhotoImage(arrow_image)
    down_photo = ImageTk.PhotoImage(arrow_image.rotate(180))
    right_photo = ImageTk.PhotoImage(arrow_image.rotate(270))
    left_photo = ImageTk.PhotoImage(arrow_image.rotate(90))

    # Define paths for servo status icons
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
    canvas.create_text(750, 100, text="SERVOS", font=font_title, fill="White", anchor="center")

    # Motor 1
    canvas.create_text(600, 180, text="SERVO 1", font=font_2, fill="White", anchor="center")
    arrowleft1 = canvas.create_image(475, 215, image=left_photo, anchor="center")
    arrowright1 = canvas.create_image(725, 215, image=right_photo, anchor="center")

    # Motor 2
    canvas.create_text(600, 255, text="SERVO 2", font=font_2, fill="White", anchor="center")
    arrowleft2 = canvas.create_image(475, 290, image=left_photo, anchor="center")
    arrowright2 = canvas.create_image(725, 290, image=right_photo, anchor="center")

    # Motor 3
    canvas.create_text(600, 330, text="SERVO 3", font=font_2, fill="White", anchor="center")
    arrowleft3 = canvas.create_image(475, 365, image=left_photo, anchor="center")
    arrowright3 = canvas.create_image(725, 365, image=right_photo, anchor="center")

    # Motor 4
    canvas.create_text(600, 405, text="SERVO 4", font=font_2, fill="White", anchor="center")
    arrowleft4 = canvas.create_image(475, 440, image=left_photo, anchor="center")
    arrowright4 = canvas.create_image(725, 440, image=right_photo, anchor="center")

    # Motor 5
    canvas.create_text(900, 180, text="SERVO 5", font=font_2, fill="White", anchor="center")
    arrowleft5 = canvas.create_image(775, 215, image=left_photo, anchor="center")
    arrowright5 = canvas.create_image(1025, 215, image=right_photo, anchor="center")

    # Motor 6
    canvas.create_text(900, 255, text="SERVO 6", font=font_2, fill="White", anchor="center")
    arrowleft6 = canvas.create_image(775, 290, image=left_photo, anchor="center")
    arrowright6 = canvas.create_image(1025, 290, image=right_photo, anchor="center")

    # Motor 7
    canvas.create_text(900, 330, text="SERVO 7", font=font_2, fill="White", anchor="center")
    arrowleft7 = canvas.create_image(775, 365, image=left_photo, anchor="center")
    arrowright7 = canvas.create_image(1025, 365, image=right_photo, anchor="center")

    # Motor 8
    canvas.create_text(900, 405, text="SERVO 8", font=font_2, fill="White", anchor="center")
    arrowleft8 = canvas.create_image(775, 440, image=left_photo, anchor="center")
    arrowright8 = canvas.create_image(1025, 440, image=right_photo, anchor="center")

    # Create sliders for 8 servos
    create_slider(canvas, 600, 215, "S01", change_slider_value)
    create_slider(canvas, 600, 290, "S02", change_slider_value)
    create_slider(canvas, 600, 365, "S03", change_slider_value)
    create_slider(canvas, 600, 440, "S04", change_slider_value)
    create_slider(canvas, 900, 215, "S05", change_slider_value)
    create_slider(canvas, 900, 290, "S06", change_slider_value)
    create_slider(canvas, 900, 365, "S07", change_slider_value)
    create_slider(canvas, 900, 440, "S08", change_slider_value)

    # Binding button presses for continuous action for each motor
    canvas.tag_bind(arrowleft1, "<ButtonPress-1>", lambda e: on_button_press("S01", -1))
    canvas.tag_bind(arrowright1, "<ButtonPress-1>", lambda e: on_button_press("S01", 1))
    canvas.tag_bind(arrowleft2, "<ButtonPress-1>", lambda e: on_button_press("MS02", -1))
    canvas.tag_bind(arrowright2, "<ButtonPress-1>", lambda e: on_button_press("S02", 1))
    canvas.tag_bind(arrowleft3, "<ButtonPress-1>", lambda e: on_button_press("S03", -1))
    canvas.tag_bind(arrowright3, "<ButtonPress-1>", lambda e: on_button_press("S03", 1))
    canvas.tag_bind(arrowleft4, "<ButtonPress-1>", lambda e: on_button_press("S04", -1))
    canvas.tag_bind(arrowright4, "<ButtonPress-1>", lambda e: on_button_press("S04", 1))
    canvas.tag_bind(arrowleft5, "<ButtonPress-1>", lambda e: on_button_press("S05", -1))
    canvas.tag_bind(arrowright5, "<ButtonPress-1>", lambda e: on_button_press("S05", 1))
    canvas.tag_bind(arrowleft6, "<ButtonPress-1>", lambda e: on_button_press("S06", -1))
    canvas.tag_bind(arrowright6, "<ButtonPress-1>", lambda e: on_button_press("S06", 1))
    canvas.tag_bind(arrowleft7, "<ButtonPress-1>", lambda e: on_button_press("S07", -1))
    canvas.tag_bind(arrowright7, "<ButtonPress-1>", lambda e: on_button_press("S07", 1))
    canvas.tag_bind(arrowleft8, "<ButtonPress-1>", lambda e: on_button_press("S08", -1))
    canvas.tag_bind(arrowright8, "<ButtonPress-1>", lambda e: on_button_press("S08", 1))

    # Stop on release for each motor
    canvas.tag_bind(arrowleft1, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright1, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft2, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright2, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft3, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright3, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft4, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright4, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft5, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright5, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft6, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright6, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft7, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright7, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowleft8, "<ButtonRelease-1>", on_button_release)
    canvas.tag_bind(arrowright8, "<ButtonRelease-1>", on_button_release)