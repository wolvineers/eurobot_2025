import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
from frames.servo_frame import servo_frame
from frames.motor_frame import motor_frame
from frames.camera_frame import camera_frame

#Set the function to design the frame03 (Manual Control Frame)
def frame03(canvas):

    global servo_photo, camera_photo, motor_photo, camera_button
    from frames.main_frame import switch_frame

    def switch_to_servo_frame():
        switch_frame(servo_frame, "SERVO CONFIGURATION")

    def switch_to_motor_frame():
        switch_frame(motor_frame, "MOTOR CONFIGURATION")

    def switch_to_camera_frame():
        switch_frame(camera_frame, "CAMERA CONFIGURATION")

    servo_image = Image.open("eurobot_ws/src/gui/img/servo_white.png")
    servo_image = servo_image.resize((150, 150), Image.LANCZOS)
    servo_photo = ImageTk.PhotoImage(servo_image)

    camera_image = Image.open("eurobot_ws/src/gui/img/camera_white.png")
    camera_image = camera_image.resize((150, 150), Image.LANCZOS)
    camera_photo = ImageTk.PhotoImage(camera_image)

    motor_image = Image.open("eurobot_ws/src/gui/img/motor_white.png")
    motor_image = motor_image.resize((150, 150), Image.LANCZOS)
    motor_photo = ImageTk.PhotoImage(motor_image)

    servo_button = canvas.create_image(690, 100, image=servo_photo, anchor="nw")  
    camera_button = canvas.create_image(900, 300, image=camera_photo, anchor="nw")
    motor_button = canvas.create_image(500, 300, image=motor_photo, anchor="nw")  

    canvas.tag_bind(servo_button, "<Button-1>", lambda e: switch_to_servo_frame())
    canvas.tag_bind(camera_button, "<Button-1>", lambda e: switch_to_camera_frame())
    canvas.tag_bind(motor_button, "<Button-1>", lambda e: switch_to_motor_frame())
    




