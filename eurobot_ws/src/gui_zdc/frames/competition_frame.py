import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
from frames.strategy_frame import strategy_frame
from frames.trajectory_frame import trajectory_frame
# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def competition_frame(canvas):
    """ 
    Set the function to design the Competition Frame

    Args:
        (canvas): Variable that sets the shape of the window
    """
    global button_photo
    from frames.main_frame import switch_frame
    
    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((250, 35), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    # Create and place images and text on the canvas
    canvas.create_text(750, 100, text="COMPETITION", font=font_title, fill="White", anchor="center")

    canvas.create_text(550, 250, text="0", font=numbers_big, fill="White", anchor="center")
    canvas.create_text(850, 250, text="0:00", font=numbers_big, fill="White", anchor="center")

    canvas.create_text(550, 350, text="POINTS", font=font_1, fill="White", anchor="center")

    canvas.create_text(850, 355, text="TIME LEFT", font=font_1, fill="White", anchor="center")

    img_traj = canvas.create_image(600, 425, image=button_photo, anchor="center")
    txt_traj = canvas.create_text(600, 425, text="SEE TRAJECTORY", font=font_2, fill="White", anchor="center")

    #TO-DO: PROGRAM FUNCTIONALITY
    canvas.create_image(900, 425, image=button_photo, anchor="center")
    canvas.create_text(900, 425, text="INITIALIZE", font=font_2, fill="White", anchor="center")

    img_strategy = canvas.create_image(750, 500, image=button_photo, anchor="center")
    txt_strategy = canvas.create_text(750, 500, text="CHANGE STRATEGY", font=font_2, fill="White", anchor="center")

    canvas.tag_bind(img_traj, "<Button-1>", lambda e: switch_frame(trajectory_frame))
    canvas.tag_bind(txt_traj, "<Button-1>", lambda e: switch_frame(trajectory_frame))

    canvas.tag_bind(img_strategy, "<Button-1>", lambda e: switch_frame(trajectory_frame))
    canvas.tag_bind(txt_strategy, "<Button-1>", lambda e: switch_frame(strategy_frame))
