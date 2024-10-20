import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")


#Set the function to design the frame01 (Settings Frame)
def frame01(canvas):

    courirer_font = tkFont.Font(family="Courier", size=22)

    canvas.create_text(530, 120, text="computing zone", font=courirer_font, fill="White")

    canvas.create_text(900, 120, text="x.x.x.x", font=courirer_font, fill="White")






    
