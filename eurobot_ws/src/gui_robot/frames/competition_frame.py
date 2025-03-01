import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
from frames.strategy_frame import strategy_frame
from frames.trajectory_frame import trajectory_frame
from frames.welcome_frame import welcome_frame
# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def competition_frame(canvas):
    """ 
    Set the function to design the Competition Frame

    Args:
        (canvas): Variable that sets the shape of the window
    """
    global button_photo, background_photo, back_photo
    from frames.main_frame import switch_frame, window
    
    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    font_1 = tkFont.Font(family="Courier", size=46)
    font_2 = tkFont.Font(family="Courier", size=18)
    font_3 = tkFont.Font(family="Courier", size=36)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers = tkFont.Font(family="Orbitron", size=64)
    font_points = tkFont.Font(family="Orbitron", size=290)

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((250, 35), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

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

    # Create and place images and text on the canvas
    canvas.create_text(602, 225, text="000", font=font_points, fill="White", anchor="center")

    canvas.create_text(602, 450, text="POINTS", font=font_1, fill="White", anchor="center")

    img_traj = canvas.create_image(300, 50, image=button_photo, anchor="center")
    txt_traj = canvas.create_text(300, 54, text="SEE TRAJECTORY", font=font_2, fill="White", anchor="center")

    #TO-DO: PROGRAM FUNCTIONALITY
    canvas.create_image(600, 50, image=button_photo, anchor="center")
    canvas.create_text(600, 54, text="INITIALIZE", font=font_2, fill="White", anchor="center")

    img_strategy = canvas.create_image(900, 50, image=button_photo, anchor="center")
    txt_strategy = canvas.create_text(900, 54, text="CHANGE STRATEGY", font=font_2, fill="White", anchor="center")

    canvas.create_text(600, 530, text="TIME REST: xx:xx", font=font_3, fill="White", anchor="center")

    canvas.tag_bind(img_traj, "<Button-1>", lambda e: switch_frame(trajectory_frame))
    canvas.tag_bind(txt_traj, "<Button-1>", lambda e: switch_frame(trajectory_frame))

    canvas.tag_bind(img_strategy, "<Button-1>", lambda e: switch_frame(trajectory_frame))
    canvas.tag_bind(txt_strategy, "<Button-1>", lambda e: switch_frame(strategy_frame))

    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(welcome_frame))