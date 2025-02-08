import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))


def frame02(canvas):
    """ 
    Set the function to design the frame02 (Cometition Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global field_photo, frame_photo # Set the global variables

    courirer_font = tkFont.Font(family="Courier", size=22) # Set the font for the default text
   
    # Set the variable that contains the path of the image
    field_path = os.path.join(current_directory, "../img/field.png")
    field_image = Image.open(field_path)
    field_image = field_image.resize((400, 250), Image.LANCZOS)
    field_photo = ImageTk.PhotoImage(field_image)

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((440, 280), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    # Display all the content (text and images) on the frame
    canvas.create_image(560, 100, image=field_photo, anchor="nw")
    canvas.create_image(540, 85, image=frame_photo, anchor="nw")

    canvas.create_text(530, 415, text="main robot", font=courirer_font, fill="White")
    canvas.create_text(540, 465, text="start robot", font=courirer_font, fill="White")
    canvas.create_text(540, 515, text="."*3, font=courirer_font, fill="White")

    canvas.create_text(900, 415, text="x.x.x.x", font=courirer_font, fill="White")
    canvas.create_text(900, 465, text="x.x.x.x", font=courirer_font, fill="White")
    canvas.create_text(900, 515, text="x.x.x.x", font=courirer_font, fill="White")





    
