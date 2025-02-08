import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
from frames.groups_frame import groups_frame
from frames.servo_frame import servo_frame
from frames.motor_frame import motor_frame


# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def control_zone_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo

    from frames.main_frame import switch_frame

    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="CONTROL ZONE", font=font_title, fill="White", anchor="center")
   
    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((300, 60), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    img_servos = canvas.create_image(750, 225, image=button_photo, anchor="center")
    txt_servos = canvas.create_text(750, 225, text="SERVOS", font=font_1, fill="White", anchor="center")

    img_motors = canvas.create_image(750, 325, image=button_photo, anchor="center")
    txt_motors = canvas.create_text(750, 325, text="MOTORS", font=font_1, fill="White", anchor="center")

    img_groups = canvas.create_image(750, 425, image=button_photo, anchor="center")
    txt_groups = canvas.create_text(750, 425, text="GROUPS", font=font_1, fill="White", anchor="center")

    canvas.tag_bind(img_servos, "<Button-1>", lambda e: switch_frame(servo_frame))
    canvas.tag_bind(txt_servos, "<Button-1>", lambda e: switch_frame(servo_frame))

    canvas.tag_bind(img_motors, "<Button-1>", lambda e: switch_frame(motor_frame))
    canvas.tag_bind(txt_motors, "<Button-1>", lambda e: switch_frame(motor_frame))

    canvas.tag_bind(img_groups, "<Button-1>", lambda e: switch_frame(groups_frame))
    canvas.tag_bind(txt_groups, "<Button-1>", lambda e: switch_frame(groups_frame))