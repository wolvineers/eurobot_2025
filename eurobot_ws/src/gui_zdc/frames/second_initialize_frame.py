import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk
from frames.first_initialize_frame import first_initialize_frame

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def start_competition(canvas):
    from frames.main_frame import switch_frame
    from frames.third_initialize_frame import third_initialize_frame

    switch_frame(third_initialize_frame, True)  # Change to the competition frame

def second_initialize_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo, background_photo, back_photo, frame_photo, field_photo, forward_photo

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
    forward_photo = ImageTk.PhotoImage(back_image.rotate(180))
    back_photo = ImageTk.PhotoImage(back_image)


    background_path = os.path.join(current_directory, "../img/background.jpg")
    background_image = Image.open(background_path)
    background_image = background_image.resize((int(screen_width), int(screen_height)), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)

    canvas.create_image(0, 0, image=background_photo, anchor="nw")
    img_back = canvas.create_image(24, 24, image=back_photo, anchor="nw")
    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(first_initialize_frame))

    img_forward = canvas.create_image(1176, 24, image=forward_photo, anchor="ne")
    canvas.tag_bind(img_forward, "<Button-1>", lambda e: start_competition(canvas))


    canvas.create_text(600, 50, text="● ● ○ ○", font=font_title, fill="White", anchor="center")
    canvas.create_text(600, 110, text="TRAJECTORY", font=font_title, fill="White", anchor="center")
    
    # Set the variable that contains the path of the image
    field_path = os.path.join(current_directory, "../img/field.png")
    field_image = Image.open(field_path)
    field_image = field_image.resize((500, 318), Image.LANCZOS)
    field_photo = ImageTk.PhotoImage(field_image)

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((550, 348), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    # Display all the content (text and images) on the frame
    canvas.create_image(600, 340, image=field_photo, anchor="center")
    canvas.create_image(600, 340, image=frame_photo, anchor="center")

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((350, 75), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)