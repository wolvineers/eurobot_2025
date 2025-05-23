import tkinter.font as tkFont
import os
from PIL import Image, ImageTk

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None


def panel_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global green_photo, red_photo, green_square_photo, frame_photo  # Set the global variables
    from frames.main_frame import tk

    courirer_font = tkFont.Font(family="Courier", size=35) # Set the font for the default text
    esp_font = tkFont.Font(family="Courier", size=20) # Set the font for the ESP text

    # Set the variable that contains the path of the image
    green_path = os.path.join(current_directory, "../img/green.png") 
    green_image = Image.open(green_path)
    green_image = green_image.resize((16, 16), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_path = os.path.join(current_directory, "../img/red.png")
    red_image = Image.open(red_path)
    red_image = red_image.resize((16, 16), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)
    
    font_1 = tkFont.Font(family="Courier", size=32)
    font_2 = tkFont.Font(family="Courier", size=28)
    font_3 = tkFont.Font(family="Courier", size=22)
    font_4 = tkFont.Font(family="Courier", size=10)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=96)

    canvas.create_text(750, 100, text="PANEL", font=font_title, fill="White", anchor="center")

    canvas.create_text(600, 150, text="SENSORS", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 180, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 200, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 220, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 240, text="Sensors 1", font=font_3, fill="White", anchor="e")

    canvas.create_image(625, 180, image=green_photo, anchor="w")
    canvas.create_image(625, 200, image=red_photo, anchor="w")
    canvas.create_image(625, 220, image=red_photo, anchor="w")
    canvas.create_image(625, 240, image=red_photo, anchor="w")

    canvas.create_text(600, 290, text="BUTTONS", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 320, text="STOP", font=font_3, fill="White", anchor="e")
    canvas.create_image(625, 320, image=red_photo, anchor="w")
    canvas.create_text(600, 340, text="Button 2", font=font_3, fill="White", anchor="e")
    canvas.create_image(625, 340, image=red_photo, anchor="w")

    canvas.create_text(600, 380, text="SIMA", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 410, text="Sima 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 430, text="Sima 2", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 450, text="Sima 3", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 470, text="Sima 4", font=font_3, fill="White", anchor="e")

    canvas.create_image(625, 410, image=green_photo, anchor="w")
    canvas.create_image(625, 430, image=red_photo, anchor="w")
    canvas.create_image(625, 450, image=red_photo, anchor="w")
    canvas.create_image(625, 470, image=red_photo, anchor="w")

    canvas.create_text(775, 380, text="SIMA", font=font_1, fill="White", anchor="e")
    canvas.create_text(775, 410, text="Sima 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(775, 430, text="Sima 2", font=font_3, fill="White", anchor="e")
    canvas.create_text(775, 450, text="Sima 3", font=font_3, fill="White", anchor="e")
    canvas.create_text(775, 470, text="Sima 4", font=font_3, fill="White", anchor="e")

    canvas.create_image(800, 410, image=green_photo, anchor="w")
    canvas.create_image(800, 430, image=red_photo, anchor="w")
    canvas.create_image(800, 450, image=red_photo, anchor="w")
    canvas.create_image(800, 470, image=red_photo, anchor="w")

    canvas.create_text(950, 380, text="SIMA", font=font_1, fill="White", anchor="e")
    canvas.create_text(950, 410, text="Sima 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(950, 430, text="Sima 2", font=font_3, fill="White", anchor="e")
    canvas.create_text(950, 450, text="Sima 3", font=font_3, fill="White", anchor="e")
    canvas.create_text(950, 470, text="Sima 4", font=font_3, fill="White", anchor="e")

    canvas.create_image(975, 410, image=green_photo, anchor="w")
    canvas.create_image(975, 430, image=red_photo, anchor="w")
    canvas.create_image(975, 450, image=red_photo, anchor="w")
    canvas.create_image(975, 470, image=red_photo, anchor="w")

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((320, 195), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)
    canvas.create_image(665, 140, image=frame_photo, anchor="nw")

    data_text = tk.Text(
        canvas,
        width=48, height=17, font=font_4, wrap=tk.WORD, fg="white", bg="black")
    
    canvas.create_window(675, 150, window=data_text, anchor="nw")
