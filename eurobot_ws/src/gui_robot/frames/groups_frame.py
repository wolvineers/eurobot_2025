import tkinter.font as tkFont
import os
from PIL import Image, ImageTk

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def groups_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo, up_photo, down_photo, left_photo, right_photo  # Set the global variables

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

    canvas.create_text(750, 100, text="GROUPS", font=font_title, fill="White", anchor="center")

    canvas.create_text(450, 200, text="Group 1", font=font_1, fill="White", anchor="w")
    canvas.create_text(450, 300, text="Group 2", font=font_1, fill="White", anchor="w")
    canvas.create_text(450, 400, text="Group 3", font=font_1, fill="White", anchor="w")

    canvas.create_image(850, 200, image=up_photo, anchor="center")
    canvas.create_image(950, 200, image=down_photo, anchor="center")

    canvas.create_image(850, 300, image=left_photo, anchor="center")
    canvas.create_image(950, 300, image=right_photo, anchor="center")

    canvas.create_image(900, 400, image=button_photo, anchor="center")
    canvas.create_text(900, 400, text="MOVE", font=font_2, fill="White", anchor="center")
