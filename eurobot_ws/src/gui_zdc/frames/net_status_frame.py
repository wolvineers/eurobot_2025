import tkinter.font as tkFont
import os
from PIL import Image, ImageTk



# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

def net_status_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global green_photo, red_photo, green_square_photo  # Set the global variables

    courirer_font = tkFont.Font(family="Courier", size=35) # Set the font for the default text
    esp_font = tkFont.Font(family="Courier", size=20) # Set the font for the ESP text

    # Set the variable that contains the path of the image
    green_path = os.path.join(current_directory, "../img/green.png") 
    green_image = Image.open(green_path)
    green_image = green_image.resize((24, 24), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((24, 24), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)
    
    font_1 = tkFont.Font(family="Courier", size=32)
    font_2 = tkFont.Font(family="Courier", size=28)
    font_3 = tkFont.Font(family="Orbitron", size=22)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=96)

    canvas.create_text(750, 100, text="NET STATUS", font=font_title, fill="White", anchor="center")

    canvas.create_text(650, 190, text="SIMA STATUS", font=font_1, fill="White", anchor="e")
    canvas.create_text(650, 230, text="127.0.0.1", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 260, text="127.0.0.1", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 290, text="127.0.0.1", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 320, text="127.0.0.1", font=font_2, fill="White", anchor="e")

    canvas.create_image(675, 230, image=green_photo, anchor="w")
    canvas.create_image(675, 260, image=red_photo, anchor="w")
    canvas.create_image(675, 290, image=red_photo, anchor="w")
    canvas.create_image(675, 320, image=red_photo, anchor="w")

    canvas.create_text(650, 380, text="Robot STATUS", font=font_1, fill="White", anchor="e")
    canvas.create_text(650, 420, text="127.0.0.1", font=font_2, fill="White", anchor="e")
    canvas.create_image(675, 420, image=red_photo, anchor="w")

    canvas.create_text(650, 480, text="ZdC STATUS", font=font_1, fill="White", anchor="e")
    canvas.create_text(650, 510, text="127.0.0.1", font=font_2, fill="White", anchor="e")
    canvas.create_image(675, 510, image=green_photo, anchor="w")

    canvas.create_text(1050, 300, text="66%", font=numbers_big, fill="White", anchor="e")

    canvas.create_text(1050, 385, text="of the devices \nare running", font=font_3, fill="White", anchor="e")
