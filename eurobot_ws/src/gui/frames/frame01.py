import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")


def frame01(canvas):
    """ 
    Set the function to design the frame01 (Settings Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    courirer_font = tkFont.Font(family="Courier", size=22) # Set the font for the default text

    # Display all the content (images) on the frame
    canvas.create_text(530, 120, text="computing zone", font=courirer_font, fill="White")
    canvas.create_text(900, 120, text="x.x.x.x", font=courirer_font, fill="White")






    
