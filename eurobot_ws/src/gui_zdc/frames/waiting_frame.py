import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk


# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def waiting_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global background_photo
    #Set the shape of the window
    canvas.image_cache = {}
    from frames.main_frame import window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    font_1 = tkFont.Font(family="Courier", size=42)
    font_2 = tkFont.Font(family="Courier", size=52)
    font_3 = tkFont.Font(family="Courier", size=32)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=75)


    background_path = os.path.join(current_directory, "../img/background.jpg")
    background_image = Image.open(background_path)
    background_image = background_image.resize((int(screen_width), int(screen_height)), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)
    canvas.create_image(0, 0, image=background_photo, anchor="nw")

    # Create the text item and store the ID
    text_id = canvas.create_text(130, 300, text="READY TO START.", font=font_title, fill="White", anchor="w")

    # Animation state
    dots = [".", "..", "..."]
    state = {"index": 0}

    def animate():
        new_text = f"READY TO START{dots[state['index']]}"
        canvas.itemconfig(text_id, text=new_text)
        state["index"] = (state["index"] + 1) % len(dots)
        canvas.after(200, animate)

    animate()