import tkinter as tk
import tkinter.font as tkFont
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
def set_servo_velocity(value, servo):
    port = '/dev/ttyUSB0'
    baudrate = 115200

    serial_port = open_serial_port(port, baudrate)

    if not serial_port:
        print("Could not open serial port.")
        return 

    print(f"Position {servo}: {value}%")

    send_message(serial_port, servo+","+value)


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
def create_slider(canvas, x, y, servo_id):
    slider = tk.Scale(
        canvas,
        from_=0,
        to=360,
        orient=tk.HORIZONTAL,
        length=200,
        width=15,
        bd=0,
        command=lambda value: set_servo_velocity(value, servo_id),
    )
    canvas.create_window(x, y, window=slider, anchor="center")
    servo_sliders[servo_id] = slider  # Store the slider in the global mapping
    return slider

# Function to set the servo to a specific angle
def set_servo_angle(servo, angle):
    if servo in servo_slider_values:
        # Update the slider value visually
        servo_slider_values[servo] = angle
        slider = servo_sliders.get(servo)
        if slider:
            slider.set(angle)  # Update the slider visually
        # Send the message to set the servo to the desired angle
        set_servo_velocity(angle, servo)

# Function to handle the servo selection and angle setting
def on_angle_button_press():
    selected_servo = servo_var.get()
    if selected_servo:
        if selected_servo in servo_slider_values:
            # Setting the angle based on the button clicked
            if current_angle == 0:
                set_servo_angle(selected_servo, 0)
            elif current_angle == 90:
                set_servo_angle(selected_servo, 90)
            elif current_angle == 180:
                set_servo_angle(selected_servo, 180)
            elif current_angle == 270:
                set_servo_angle(selected_servo, 270)

# Create the drop-down menu to select the servo
def create_servo_selector(canvas, x, y):
    global servo_var  # Define the global variable for the selected servo

    servo_var = tk.StringVar()
    servo_var.set("S01")  # Default selected servo

    # Create the OptionMenu (drop-down selector)
    servo_selector = tk.OptionMenu(canvas, servo_var, *servo_slider_values.keys())
    canvas.create_window(x, y, window=servo_selector, anchor="center")

# Set angle buttons for 0º, 90º, 180º, 270º
def create_angle_buttons(canvas, x, y):
    global current_angle
    current_angle = 0  # Default angle set to 0º

    button1 = canvas.create_image(x + 100, y, image=button_photo, anchor="center")
    text1 = canvas.create_text(x + 100, y, text="0º", font=tkFont.Font(family="Courier", size=20), fill="White", anchor="center")

    canvas.tag_bind(button1, "<Button-1>", lambda e: set_angle(0))
    canvas.tag_bind(text1, "<Button-1>", lambda e: set_angle(0))

    
    button2 = canvas.create_image(x + 200, y, image=button_photo, anchor="center")
    text2 = canvas.create_text(x + 200, y, text="90º", font=tkFont.Font(family="Courier", size=20), fill="White", anchor="center")

    
    canvas.tag_bind(button2, "<Button-1>", lambda e: set_angle(90))
    canvas.tag_bind(text2, "<Button-1>", lambda e: set_angle(90))

    button3 = canvas.create_image(x + 300, y, image=button_photo, anchor="center")
    text3 = canvas.create_text(x + 300, y, text="180º", font=tkFont.Font(family="Courier", size=20), fill="White", anchor="center")

    canvas.tag_bind(button3, "<Button-1>", lambda e: set_angle(180))
    canvas.tag_bind(text3, "<Button-1>", lambda e: set_angle(180))

    
    button4 = canvas.create_image(x + 400, y, image=button_photo, anchor="center")
    text4 = canvas.create_text(x + 400, y, text="270º", font=tkFont.Font(family="Courier", size=20), fill="White", anchor="center")

    
    canvas.tag_bind(button4, "<Button-1>", lambda e: set_angle(270))
    canvas.tag_bind(text4, "<Button-1>", lambda e: set_angle(270))

# Function to set the selected servo's angle
def set_angle(angle):
    global current_angle
    current_angle = angle
    on_angle_button_press()

def servo_frame(canvas_ref):
    global canvas, button_photo, img_back, background_photo, back_photo  # Set the global variables
    canvas = canvas_ref  # Assign the canvas to the global variable

    from frames.main_frame import switch_frame, window
    from frames.control_zone_frame import control_zone_frame
    
    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((75, 35), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=48)
    font_2_orb = tkFont.Font(family="Orbitron", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=76)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

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

    # Creating text and arrows
    font_title = tkFont.Font(family="Courier", size=64)
    canvas.create_text(600, 50, text="SERVOS", font=font_title, fill="White", anchor="center")

     # Motor 1
    canvas.create_text(250, 175, text="SERVO 1", font=font_2, fill="White", anchor="center")
    
    # Motor 2
    canvas.create_text(250, 250, text="SERVO 2", font=font_2, fill="White", anchor="center")
    
    # Motor 3
    canvas.create_text(250, 325, text="SERVO 3", font=font_2, fill="White", anchor="center")
    
    # Motor 4
    canvas.create_text(250, 400, text="SERVO 4", font=font_2, fill="White", anchor="center")
    
    # Motor 5
    canvas.create_text(550, 175, text="SERVO 5", font=font_2, fill="White", anchor="center")
    
    # Motor 6
    canvas.create_text(550, 250, text="SERVO 6", font=font_2, fill="White", anchor="center")
    
    # Motor 7
    canvas.create_text(550, 325, text="SERVO 7", font=font_2, fill="White", anchor="center")
    
    # Motor 8
    canvas.create_text(550, 400, text="SERVO 8", font=font_2, fill="White", anchor="center")

    # Add the servo selector dropdown menu
    create_servo_selector(canvas, 540, 525)

    # Add the angle buttons to set the servo angle
    create_angle_buttons(canvas, 550, 525)

    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(control_zone_frame))
