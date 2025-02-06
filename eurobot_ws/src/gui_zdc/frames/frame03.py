import tkinter as tk
import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
from frames.servo_frame import servo_frame
from frames.motor_frame import motor_frame
from frames.camera_frame import camera_frame

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def frame03(canvas):
    """ 
    Set the function to design the frame03 (Manual Control Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global servo_photo, camera_photo, motor_photo, camera_button # Set the global variables
    from frames.main_frame import switch_frame # Import de function to switch the frame

    # Set the functions to switch for the different options
    def switch_to_servo_frame():
        switch_frame(servo_frame, "SERVO CONFIGURATION")

    def switch_to_motor_frame():
        switch_frame(motor_frame, "MOTOR CONFIGURATION")

    def switch_to_camera_frame():
        switch_frame(camera_frame, "CAMERA CONFIGURATION")

    # Set the variable that contains the path of the image
    servo_path = os.path.join(current_directory, "../img/servo_white.png")
    servo_image = Image.open(servo_path)
    servo_image = servo_image.resize((150, 150), Image.LANCZOS)
    servo_photo = ImageTk.PhotoImage(servo_image)

    camera_path = os.path.join(current_directory, "../img/camera_white.png")
    camera_image = Image.open(camera_path)
    camera_image = camera_image.resize((150, 150), Image.LANCZOS)
    camera_photo = ImageTk.PhotoImage(camera_image)

    motor_path = os.path.join(current_directory, "../img/motor_white.png")
    motor_image = Image.open(motor_path)
    motor_image = motor_image.resize((150, 150), Image.LANCZOS)
    motor_photo = ImageTk.PhotoImage(motor_image)

    # Display all the content (text and images) on the frame
    servo_button = canvas.create_image(690, 100, image=servo_photo, anchor="nw")  
    camera_button = canvas.create_image(900, 300, image=camera_photo, anchor="nw")
    motor_button = canvas.create_image(500, 300, image=motor_photo, anchor="nw")  

    # Convert the image to a button to change the frames
    canvas.tag_bind(servo_button, "<Button-1>", lambda e: switch_to_servo_frame())
    canvas.tag_bind(camera_button, "<Button-1>", lambda e: switch_to_camera_frame())
    canvas.tag_bind(motor_button, "<Button-1>", lambda e: switch_to_motor_frame())
    




