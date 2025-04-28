import tkinter.font as tkFont
import os
from PIL import Image, ImageTk



# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def trajectory_frame(canvas):
    """ 
    Set the function to design the Trajectory Frame
    Args:
        (canvas): Variable that set the shape of the window
    """
    global field_photo, frame_photo # Set the global variables

    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="TRAJECTORY", font=font_title, fill="White", anchor="center")
   
    # Set the variable that contains the path of the image
    field_path = os.path.join(current_directory, "../img/field.png")
    field_image = Image.open(field_path)
    field_image = field_image.resize((450, 281), Image.LANCZOS)
    field_photo = ImageTk.PhotoImage(field_image)

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((500, 318), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    # Display all the content (text and images) on the frame
    canvas.create_image(750, 325, image=field_photo, anchor="center")
    canvas.create_image(750, 325, image=frame_photo, anchor="center")
