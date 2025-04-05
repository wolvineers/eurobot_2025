import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
import subprocess


# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))
from ping3 import ping

def check_ping(host):
    """Ping using ICMP (returns True if reachable)."""
    try:
        response = ping(host, timeout=1)
        return response is not None
    except Exception as e:
        print(f"ICMP ping failed for {host}: {e}")
        return False
    
def net_status_frame(canvas):
    """Design the NET STATUS frame with dynamic ping results."""

    global green_photo, red_photo

    # Fonts
    courirer_font = tkFont.Font(family="Courier", size=35)
    esp_font = tkFont.Font(family="Courier", size=20)
    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Orbitron", size=22)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=86)

    # Load images
    green_path = os.path.join(current_directory, "../img/green.png")
    red_path = os.path.join(current_directory, "../img/red.png")

    green_image = Image.open(green_path).resize((24, 24), Image.LANCZOS)
    red_image = Image.open(red_path).resize((24, 24), Image.LANCZOS)

    green_photo = ImageTk.PhotoImage(green_image)
    red_photo = ImageTk.PhotoImage(red_image)

    # Title
    canvas.create_text(750, 100, text="NET STATUS", font=font_title, fill="White", anchor="center")

    # Devices & Hosts to test
    devices = [
        ("SIMA STATUS", ["google.com", "google.com", "google.com", "google.com"]),
        ("Robot STATUS", ["google.com"]),
        ("ZdC STATUS", ["google.com"])
    ]

    total_hosts = 0
    active_hosts = 0

    start_y = 190
    spacing = 30
    image_x = 675
    text_x = 650

    for device_index, (label, hosts) in enumerate(devices):
        canvas.create_text(text_x, start_y, text=label, font=font_1, fill="White", anchor="e")
        current_y = start_y + spacing

        for host in hosts:
            is_up = check_ping(host)
            total_hosts += 1
            if is_up:
                active_hosts += 1

            canvas.create_text(text_x, current_y, text=host, font=font_2, fill="White", anchor="e")
            canvas.create_image(image_x, current_y, image=green_photo if is_up else red_photo, anchor="w")
            current_y += spacing

        start_y = current_y + 20  # Add extra space after each section

    # Calculate % of devices running
    percent_running = int((active_hosts / total_hosts) * 100) if total_hosts > 0 else 0
    percent_text = f"{percent_running}%"

    # Display the result
    canvas.create_text(1050, 300, text=percent_text, font=numbers_big, fill="White", anchor="e")
    canvas.create_text(1050, 390, text="of the devices \nare running", font=font_3, fill="White", anchor="e")