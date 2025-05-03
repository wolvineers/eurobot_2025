import tkinter.font as tkFont
import os
from PIL import Image, ImageTk



# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def frame00(canvas):
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
    green_image = green_image.resize((150, 150), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((150, 150), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)

    green_square_path = os.path.join(current_directory, "../img/green_square.png")
    green_square_image = Image.open(green_square_path)
    green_square_image = green_square_image.resize((150, 150), Image.LANCZOS)
    green_square_photo = ImageTk.PhotoImage(green_square_image)

    # Display all the content (text and images) on the frame
    canvas.create_text(750, 100, text="CONTROL SYSTEMS", font=courirer_font, fill="White")
    canvas.create_text(750, 130, text="_" * 30, font=esp_font, fill="White")

    canvas.create_text(500, 200, text="esp 01", font=esp_font, fill="White")
    canvas.create_text(500, 290, text="esp 02", font=esp_font, fill="White")
    canvas.create_text(520, 380, text="comp.zone", font=esp_font, fill="White")
    canvas.create_text(510, 470, text="battery", font=esp_font, fill="White")

    canvas.create_text(770, 200, text="x.x.x.x", font=esp_font, fill="White")
    canvas.create_text(770, 290, text="x.x.x.x", font=esp_font, fill="White")
    canvas.create_text(770, 380, text="x.x.x.x", font=esp_font, fill="White")
    canvas.create_text(770, 470, text="x%", font=esp_font, fill="White")

    canvas.create_image(950, 120, image=green_photo, anchor="nw")
    canvas.create_image(950, 210, image=red_photo, anchor="nw")
    canvas.create_image(950, 300, image=red_photo, anchor="nw")
    canvas.create_image(970, 410, image=green_square_photo, anchor="nw")
