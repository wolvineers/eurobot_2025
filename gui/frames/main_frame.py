import tkinter as tk
from PIL import Image, ImageTk
import os
import tkinter.font as tkFont
from frames.frame00 import frame00
from frames.frame01 import frame01
from frames.frame02 import frame02
from frames.frame03 import frame03  # Ya está importado

os.system("clear")

window = tk.Tk()
window.title("Interfaz con fondo y transparencias")
window.geometry("1204x600")

screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()

canvas = tk.Canvas(window, width=screen_width, height=screen_height)
canvas.pack(fill="both", expand=True)

background_photo = None
wolvi_photo = None
button_photo = None
ppal_frame_left_photo = None
ppal_frame_right_photo = None

def clear_canvas():
    canvas.delete("all")

def clear_canvas():
    canvas.delete("all")

def switch_frame(frame_function, frame_name):
    print(f"Switching to {frame_name}...")
    clear_canvas()
    main_frame()
    frame_function(canvas)
    print("Done!")

# Uso de la función generalizada
def switch_to_frame00():
    switch_frame(frame00, "HOME")

def switch_to_frame01():
    switch_frame(frame01, "SETTINGS")

def switch_to_frame02():
    switch_frame(frame02, "COMPETITION")

def switch_to_frame03():
    switch_frame(frame03, "MANUAL CONTROL")


def main_frame():
    global background_photo, wolvi_photo, button_photo, ppal_frame_left_photo, ppal_frame_right_photo

    menu_font = tkFont.Font(family="Courier", size=15)

    # Cargar y redimensionar la imagen de fondo
    background_image = Image.open("gui/img/background.jpg")
    background_image = background_image.resize((screen_width, screen_height), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)

    wolvi_image = Image.open("gui/img/wolve.png")
    wolvi_image = wolvi_image.resize((250, 300), Image.LANCZOS)
    wolvi_photo = ImageTk.PhotoImage(wolvi_image)

    # Cargar y redimensionar la imagen del botón con fondo transparente
    button_image = Image.open("gui/img/white-button.png")
    button_image = button_image.resize((250, 35), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    ppal_frame_left_image = Image.open("gui/img/ppal-frame-left.png")
    ppal_frame_left_image = ppal_frame_left_image.resize((20, 500), Image.LANCZOS)
    ppal_frame_left_photo = ImageTk.PhotoImage(ppal_frame_left_image)

    ppal_frame_right_image = Image.open("gui/img/ppal-frame-right.png")
    ppal_frame_right_image = ppal_frame_right_image.resize((20, 500), Image.LANCZOS)
    ppal_frame_right_photo = ImageTk.PhotoImage(ppal_frame_right_image)

    # Dibujar la imagen de fondo en el canvas
    canvas.create_image(0, 0, image=background_photo, anchor="nw")

    # Crear botones personalizados usando imágenes
    button1 = canvas.create_image(70, 350, image=button_photo, anchor="nw")
    button2 = canvas.create_image(70, 420, image=button_photo, anchor="nw")
    button3 = canvas.create_image(70, 490, image=button_photo, anchor="nw")
    button4 = canvas.create_image(100, 0, image=wolvi_photo, anchor="nw")

    # Añadir texto a cada botón
    text1 = canvas.create_text(195, 370, text="SETTINGS", font=menu_font, fill="White")
    text2 = canvas.create_text(195, 440, text="COMPETITION", font=menu_font, fill="White")
    text3 = canvas.create_text(195, 510, text="MANUAL CONTROL", font=menu_font, fill="White")

    # Asociar eventos de clic a los botones y el texto
    canvas.tag_bind(button1, "<Button-1>", lambda e: switch_to_frame01())
    canvas.tag_bind(text1, "<Button-1>", lambda e: switch_to_frame01())

    canvas.tag_bind(button2, "<Button-1>", lambda e: switch_to_frame02())
    canvas.tag_bind(text2, "<Button-1>", lambda e: switch_to_frame02())

    canvas.tag_bind(button3, "<Button-1>", lambda e: switch_to_frame03())
    canvas.tag_bind(text3, "<Button-1>", lambda e: switch_to_frame03())

    canvas.tag_bind(button4, "<Button-1>", lambda e: switch_to_frame00())

    canvas.create_image(400, 50, image=ppal_frame_left_photo, anchor="nw")
    canvas.create_image(1100, 50, image=ppal_frame_right_photo, anchor="nw")

main_frame()
frame00(canvas)

# Función para salir del modo pantalla completa con la tecla 'Esc'
def close_program(event):
    window.destroy()

# Asociar la tecla 'Esc' para cerrar el programa
window.bind("<Escape>", close_program)

# Iniciar el bucle de la interfaz
window.mainloop()
