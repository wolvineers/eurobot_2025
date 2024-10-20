import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")

#Set the function to design the frame02 (Competition Frame)
def frame02(canvas):
    global field_photo, frame_photo

    courirer_font = tkFont.Font(family="Courier", size=22)

    field_image = Image.open("eurobot_ws/src/gui/img/field.png")
    field_image = field_image.resize((400, 250), Image.LANCZOS)
    field_photo = ImageTk.PhotoImage(field_image)

    frame_image = Image.open("eurobot_ws/src/gui/img/frame.png")
    frame_image = frame_image.resize((440, 280), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    canvas.create_image(560, 100, image=field_photo, anchor="nw")
    canvas.create_image(540, 85, image=frame_photo, anchor="nw")

    canvas.create_text(530, 415, text="main robot", font=courirer_font, fill="White")
    canvas.create_text(540, 465, text="start robot", font=courirer_font, fill="White")
    canvas.create_text(540, 515, text="."*3, font=courirer_font, fill="White")

    canvas.create_text(900, 415, text="x.x.x.x", font=courirer_font, fill="White")
    canvas.create_text(900, 465, text="x.x.x.x", font=courirer_font, fill="White")
    canvas.create_text(900, 515, text="x.x.x.x", font=courirer_font, fill="White")





    
