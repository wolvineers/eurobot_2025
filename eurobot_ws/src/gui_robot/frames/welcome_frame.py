import tkinter as tk
import tkinter.font as tkFont
from tkinter import ttk
import os
from PIL import Image, ImageTk
import time
import psutil

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

# Function to save the selected option to a file
def save_selector_value(value):
    with open("selector_data.txt", "w") as file:
        file.write(str(value))  # Save '1' for Insomnious or '2' for Torete

# Function to load the selected option from the file
def load_selector_value():
    try:
        with open("selector_data.txt", "r") as file:
            value = file.read().strip()
            return value
    except FileNotFoundError:
        return None  # No value saved yet

def get_local_ip():
    try:
        # Use os to run ifconfig with grep and awk to extract local IP
        local_ip = os.popen("ifconfig | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}'").read().strip()
        return local_ip
    except Exception as e:
        return f"Error fetching local IP: {e}"

def get_battery_percentage():
    battery = psutil.sensors_battery()
    if battery:
        return f"{battery.percent}%"
    return "No Battery"

def welcome_frame(canvas):
    """ 
    Set the function to design the Welcome Frame

    Args:
        (canvas): Variable that set the shape of the window
    """
    global battery_photo, ip_photo  # Set the global variables

    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    # Set the variable that contains the path of the image
    battery_path = os.path.join(current_directory, "../img/icons8-battery-24.png") 
    battery_image = Image.open(battery_path)
    battery_image = battery_image.resize((36, 36), Image.LANCZOS)
    battery_photo = ImageTk.PhotoImage(battery_image)

    # Set the variable that contains the path of the image
    ip_path = os.path.join(current_directory, "../img/icons8-ip-24.png") 
    ip_image = Image.open(ip_path)
    ip_image = ip_image.resize((36, 36), Image.LANCZOS)
    ip_photo = ImageTk.PhotoImage(ip_image)

    # Center the Clock
    canvas.create_text(750, 200, text=time.strftime("%H:%M"), font=numbers_big, fill="White", anchor="center")

    # Show Local Battery (Left Side)
    canvas.create_image(500, 350, image=battery_photo, anchor="center")
    canvas.create_text(530, 350, text=f"{get_battery_percentage()} (R)", font=font_1, fill="White", anchor="w")
    
    # Show ZDC Battery (Right Side)
    canvas.create_image(850, 350, image=battery_photo, anchor="center")
    canvas.create_text(880, 350, text=f"xx% (ZDC)", font=font_1, fill="White", anchor="w")

    # Show IP Address (Centered Below Batteries)
    canvas.create_image(620, 425, image=ip_photo, anchor="center")
    canvas.create_text(650, 425, text=get_local_ip(), font=font_1, fill="White", anchor="w")

    # Define the options for the dropdown
    options = ["Insomnious", "Torete"]
    selected_value = load_selector_value()  # Load the previous selection

    # Set the default value of the dropdown
    if selected_value == "1":
        default_value = "Insomnious"
    elif selected_value == "2":
        default_value = "Torete"
    else:
        default_value = "Insomnious"

    # Crear el OptionMenu
    selector_var = tk.StringVar()
    selector_var.set(default_value)  # Esto establece el valor predeterminado

    selector = tk.OptionMenu(canvas, selector_var, *options)
    canvas.create_window(750, 525, window=selector, anchor="center")

    # Función para manejar el cambio de valor en el selector
    def on_selector_change(event):
        selected_option = selector_var.get()  # Obtenemos el valor seleccionado usando selector_var
        if selected_option == "Insomnious":
            save_selector_value("1")
        elif selected_option == "Torete":
            save_selector_value("2")

    # Asociar la función de cambio al evento del selector
    selector.bind("<Configure>", on_selector_change)  # O usa otro evento según tu necesidad
