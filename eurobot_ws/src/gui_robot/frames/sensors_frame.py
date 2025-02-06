import tkinter.font as tkFont
import os
from PIL import Image, ImageTk



# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def sensors_frame(canvas):
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
    
    font_1 = tkFont.Font(family="Courier", size=36)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="SENSORS", font=font_title, fill="White", anchor="center")

    canvas.create_text(650, 200, text="Sensor 1", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 240, text="dddd 2", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 280, text="Sensodfdfr 3", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 320, text="Sensor 4", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 360, text="Sensoddddddr 5", font=font_2, fill="White", anchor="e")
    canvas.create_text(650, 400, text="Senzadsor 6", font=font_2, fill="White", anchor="e")

    canvas.create_image(675, 200, image=green_photo, anchor="w")
    canvas.create_image(675, 240, image=red_photo, anchor="w")
    canvas.create_image(675, 280, image=red_photo, anchor="w")
    canvas.create_image(675, 320, image=red_photo, anchor="w")
    canvas.create_image(675, 360, image=green_photo, anchor="w")
    canvas.create_image(675, 400, image=red_photo, anchor="w")

    canvas.create_text(850, 200, text="Sensor 7", font=font_2, fill="White", anchor="w")
    canvas.create_text(850, 240, text="Sensdsfor 8", font=font_2, fill="White", anchor="w")
    canvas.create_text(850, 280, text="Sensor 9", font=font_2, fill="White", anchor="w")
    canvas.create_text(850, 320, text="sor 10", font=font_2, fill="White", anchor="w")
    canvas.create_text(850, 360, text="Sensor 11", font=font_2, fill="White", anchor="w")
    canvas.create_text(850, 400, text="Senasdfsor 12", font=font_2, fill="White", anchor="w")

    canvas.create_image(825, 200, image=green_photo, anchor="e")
    canvas.create_image(825, 240, image=red_photo, anchor="e")
    canvas.create_image(825, 280, image=red_photo, anchor="e")
    canvas.create_image(825, 320, image=red_photo, anchor="e")
    canvas.create_image(825, 360, image=green_photo, anchor="e")
    canvas.create_image(825, 400, image=red_photo, anchor="e")
