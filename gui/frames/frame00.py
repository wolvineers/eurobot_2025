import tkinter.font as tkFont
from PIL import Image, ImageTk

# Declarar las imágenes como variables globales
green_photo = None
red_photo = None
green_square_photo = None

def frame00(canvas):
    global green_photo, red_photo, green_square_photo  # Usar las variables globales para las imágenes

    courirer_font = tkFont.Font(family="Courier", size=35)
    esp_font = tkFont.Font(family="Courier", size=20)

    green_image = Image.open("gui/img/green.png")
    green_image = green_image.resize((150, 150), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_image = Image.open("gui/img/red.png")
    red_image = red_image.resize((150, 150), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)

    green_square_image = Image.open("gui/img/green_square.png")
    green_square_image = green_square_image.resize((150, 150), Image.LANCZOS)
    green_square_photo = ImageTk.PhotoImage(green_square_image)

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
