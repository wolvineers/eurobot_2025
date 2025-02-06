import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import os
from frames.competition_frame import competition_frame
from frames.welcome_frame import welcome_frame
from frames.control_zone_frame import control_zone_frame
from frames.groups_frame import groups_frame
from frames.motor_frame import motor_frame
from frames.sensors_frame import sensors_frame
from frames.strategy_frame import strategy_frame
from frames.trajectory_frame import trajectory_frame

os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

#Start the window for the interface
window = tk.Tk()
window.title("Aplicació Pantalla Robot")
window.geometry("1204x600")
    #window.attributes('-fullscreen', True)

#Set the shape of the window
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()

#Set the canvas to add the widgets
canvas = tk.Canvas(window, width=screen_width, height=screen_height)
canvas.pack(fill="both", expand=True)

#Set all the global variables
background_photo = None
wolvi_photo = None
button_photo = None
ppal_frame_left_photo = None
ppal_frame_right_photo = None
close_photo = None

#Set the function to switch the frames
def switch_frame(frame_function):
    print(f"Switching to {frame_function}...")
    canvas.delete("all")
    main_frame()
    frame_function(canvas)
    print("Done!")

#Set the main function (The main design for the interface)
def main_frame():
    global background_photo, wolvi_photo, button_photo, ppal_frame_left_photo, ppal_frame_right_photo, close_photo

    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    numbers_big = tkFont.Font(family="Orbitron", size=14)


    background_path = os.path.join(current_directory, "../img/background.jpg")
    background_image = Image.open(background_path)
    background_image = background_image.resize((int(screen_width*0.5), int(screen_height*0.5)), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)

    close_path = os.path.join(current_directory, "../img/icons8-close-24.png")
    close_image = Image.open(close_path)
    close_image = close_image.resize((36, 36), Image.LANCZOS)
    close_photo = ImageTk.PhotoImage(close_image)


    wolvi_path = os.path.join(current_directory, "../img/wolve.png")
    wolvi_image = Image.open(wolvi_path)
    wolvi_image = wolvi_image.resize((250, 300), Image.LANCZOS)
    wolvi_photo = ImageTk.PhotoImage(wolvi_image)

    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((250, 45), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    ppal_frame_left_path = os.path.join(current_directory, "../img/ppal-frame-left.png")
    ppal_frame_left_image = Image.open(ppal_frame_left_path)
    ppal_frame_left_image = ppal_frame_left_image.resize((20, 500), Image.LANCZOS)
    ppal_frame_left_photo = ImageTk.PhotoImage(ppal_frame_left_image)

    ppal_frame_right_path = os.path.join(current_directory, "../img/ppal-frame-right.png")
    ppal_frame_right_image = Image.open(ppal_frame_right_path)
    ppal_frame_right_image = ppal_frame_right_image.resize((20, 500), Image.LANCZOS)
    ppal_frame_right_photo = ImageTk.PhotoImage(ppal_frame_right_image)

    canvas.create_image(0, 0, image=background_photo, anchor="nw")

    button1 = canvas.create_image(70, 350, image=button_photo, anchor="nw")
    button2 = canvas.create_image(70, 420, image=button_photo, anchor="nw")
    button4 = canvas.create_image(70, 0, image=wolvi_photo, anchor="nw")
    button5 = canvas.create_image(24, 24, image=close_photo, anchor="nw")

    text1 = canvas.create_text(195, 372, text="COMPETITION", font=font_2, fill="White")
    text2 = canvas.create_text(195, 442, text="CONTROL ZONE", font=font_2, fill="White")

    canvas.tag_bind(button1, "<Button-1>", lambda e: switch_frame(competition_frame))
    canvas.tag_bind(text1, "<Button-1>", lambda e: switch_frame(competition_frame))

    canvas.tag_bind(button2, "<Button-1>", lambda e: switch_frame(control_zone_frame))
    canvas.tag_bind(text2, "<Button-1>", lambda e: switch_frame(control_zone_frame))

    canvas.tag_bind(button4, "<Button-1>", lambda e: switch_frame(welcome_frame))

    canvas.tag_bind(button5, "<Button-1>", lambda e: close_program())

    canvas.create_image(400, 50, image=ppal_frame_left_photo, anchor="ne")
    canvas.create_image(1100, 50, image=ppal_frame_right_photo, anchor="nw")

#Set the function to close the interface
def close_program():
    window.destroy()

window.bind("<Escape>", close_program)

main_frame()
switch_frame(welcome_frame)

window.mainloop()
