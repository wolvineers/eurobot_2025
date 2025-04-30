import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk
import threading
from frames.gui_node import get_node  # Aquest import assumeix que la instància de gui_node és global

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def start_competition(canvas):
    from frames.main_frame import switch_frame
    from frames.second_initialize_frame import second_initialize_frame
    ros_node = get_node()  # Exemples d'ús del node ROS des de la interfície
    ros_node.initialize_robot(1)
    switch_frame(second_initialize_frame, True)

def first_initialize_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo, background_photo, back_photo
    canvas.image_cache = {}
    from frames.main_frame import switch_frame, window

    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    font_1 = tkFont.Font(family="Courier", size=42)
    font_2 = tkFont.Font(family="Courier", size=52)
    font_3 = tkFont.Font(family="Courier", size=32)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=54)
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
    from frames.competition_frame import competition_frame
    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(competition_frame, True))


    canvas.create_text(600, 50, text="● ○ ○ ○", font=font_title, fill="White", anchor="center")
    canvas.create_text(600, 110, text="INITIALIZE", font=font_title, fill="White", anchor="center")
   
    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((600, 75), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    img_initialize = canvas.create_image(600, 350, image=button_photo, anchor="center")
    txt_initialize = canvas.create_text(600, 350, text="Start Robot", font=font_2, fill="White", anchor="center")

    canvas.tag_bind(img_initialize, "<Button-1>", lambda e: start_competition(canvas))
    canvas.tag_bind(txt_initialize, "<Button-1>", lambda e: start_competition(canvas))