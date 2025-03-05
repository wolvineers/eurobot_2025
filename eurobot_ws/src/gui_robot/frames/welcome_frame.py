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
        local_ip = os.popen("ifconfig | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}'").read().strip()
        return local_ip
    except Exception as e:
        return f"Error fetching local IP: {e}"

def get_battery_percentage():
    battery = psutil.sensors_battery()
    return f"{battery.percent}%" if battery else "No Battery"

empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

empty_checkbox_image = Image.open(empty_checkbox_path).resize((40, 40), Image.LANCZOS)
selected_checkbox_image = Image.open(selected_checkbox_path).resize((40, 40), Image.LANCZOS)

checkbox_states = {"Insomnious": False, "Torete": False}

def update_checkbox_image(canvas, selected_id, other_id):
    for checkbox_id in [selected_id, other_id]:
        image_to_display = ImageTk.PhotoImage(selected_checkbox_image if checkbox_states[checkbox_id] else empty_checkbox_image)
        canvas.itemconfig(canvas.find_withtag(checkbox_id)[0], image=image_to_display)
        canvas.image_cache[checkbox_id] = image_to_display  # Prevent garbage collection

def toggle_checkbox(selected_id, canvas):
    other_id = "Insomnious" if selected_id == "Torete" else "Torete"
    checkbox_states[selected_id] = True
    checkbox_states[other_id] = False
    update_checkbox_image(canvas, selected_id, other_id)
    selector_var.set(selected_id)
    save_selector_value("1" if selected_id == "Insomnious" else "2")

def create_checkbox(canvas, x, y, selected_id):
    empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    selected_photo = ImageTk.PhotoImage(selected_checkbox_image)
    canvas.image_cache[selected_id] = empty_photo  # Prevent garbage collection

    canvas.create_image(x, y, image=empty_photo, anchor="center", tags=selected_id)
    canvas.create_text(x + 30, y + 5, text=selected_id, font=tkFont.Font(family="Courier", size=16), fill="White", anchor="w")
    canvas.tag_bind(selected_id, "<Button-1>", lambda e: toggle_checkbox(selected_id, canvas))

def on_selector_change(selected_option):
    toggle_checkbox(selected_option, canvas)

def welcome_frame(canvas):
    global selector_var
    canvas.image_cache = {}

    font_1 = tkFont.Font(family="Courier", size=24)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    battery_path = os.path.join(current_directory, "../img/icons8-battery-24.png")
    battery_image = Image.open(battery_path).resize((36, 36), Image.LANCZOS)
    battery_photo = ImageTk.PhotoImage(battery_image)
    canvas.image_cache["battery"] = battery_photo

    ip_path = os.path.join(current_directory, "../img/icons8-ip-24.png")
    ip_image = Image.open(ip_path).resize((36, 36), Image.LANCZOS)
    ip_photo = ImageTk.PhotoImage(ip_image)
    canvas.image_cache["ip"] = ip_photo

    canvas.create_text(750, 200, text=time.strftime("%H:%M"), font=numbers_big, fill="White", anchor="center")
    canvas.create_image(500, 350, image=battery_photo, anchor="center")
    canvas.create_text(530, 350, text=f"{get_battery_percentage()} (R)", font=font_1, fill="White", anchor="w")
    canvas.create_image(850, 350, image=battery_photo, anchor="center")
    canvas.create_text(880, 350, text="xx% (ZDC)", font=font_1, fill="White", anchor="w")
    canvas.create_image(620, 425, image=ip_photo, anchor="center")
    canvas.create_text(650, 425, text=get_local_ip(), font=font_1, fill="White", anchor="w")

    options = ["Insomnious", "Torete"]
    selected_value = load_selector_value()
    default_value = "Insomnious" if selected_value != "2" else "Torete"

    selector_var = tk.StringVar()
    selector_var.set(default_value)

    create_checkbox(canvas, 500, 500, "Insomnious")
    create_checkbox(canvas, 850, 500, "Torete")

    toggle_checkbox(default_value, canvas)

