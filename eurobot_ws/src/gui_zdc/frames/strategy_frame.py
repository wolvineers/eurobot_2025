import tkinter.font as tkFont
import os
from PIL import Image, ImageTk



# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def strategy_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo

    font_1 = tkFont.Font(family="Courier", size=42)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="STRATEGY", font=font_title, fill="White", anchor="center")
   
    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((600, 60), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    img_strat1 = canvas.create_image(750, 250, image=button_photo, anchor="center")
    txt_strat1 = canvas.create_text(750, 250, text="PUTEJAR STRATEGY", font=font_1, fill="White", anchor="center")

    img_strat2 = canvas.create_image(750, 350, image=button_photo, anchor="center")
    txt_strat2 = canvas.create_text(750, 350, text="MAX POINTS STRATEGY", font=font_1, fill="White", anchor="center")

    #TO-DO: PROGRAM THE FUNCTIONALITY OF THE STRATEGIES (probably send the strategy via ross)
    '''
    canvas.tag_bind(img_strat1, "<Button-1>", lambda e: )
    canvas.tag_bind(txt_strat1, "<Button-1>", lambda e: )

    canvas.tag_bind(img_strat2, "<Button-1>", lambda e: )
    canvas.tag_bind(txt_strat2, "<Button-1>", lambda e: )
    '''

