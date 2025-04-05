import tkinter.font as tkFont
import os
from PIL import Image, ImageTk

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None

import subprocess
import threading
import time

def run_ros2_commands(text_widget):
    """Run ROS2 commands every 15 seconds and update the text widget."""
    def loop():
        while True:
            try:
                node_cmd = "ros2 node list"
                topic_cmd = "ros2 topic list"

                node_output = subprocess.getoutput(node_cmd)
                topic_output = subprocess.getoutput(topic_cmd)

                output = f"$ {node_cmd}\n{node_output}\n\n$ {topic_cmd}\n{topic_output}"

                # Safely update the text widget in the main thread
                text_widget.after(0, lambda: update_text_widget(text_widget, output))

            except Exception as e:
                output = f"Error running commands:\n{e}"
                text_widget.after(0, lambda: update_text_widget(text_widget, output))

            time.sleep(15)

    threading.Thread(target=loop, daemon=True).start()

def update_text_widget(widget, text):
    widget.config(state='normal')
    widget.delete(1.0, "end")
    widget.insert("end", text)
    widget.config(state='disabled')


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
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=10)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=96)

    canvas.create_text(750, 100, text="PANEL", font=font_title, fill="White", anchor="center")

    canvas.create_text(600, 160, text="SENSORS", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 190, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 210, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 230, text="Sensors 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 250, text="Sensors 1", font=font_3, fill="White", anchor="e")

    canvas.create_image(625, 190, image=green_photo, anchor="w")
    canvas.create_image(625, 210, image=red_photo, anchor="w")
    canvas.create_image(625, 230, image=red_photo, anchor="w")
    canvas.create_image(625, 250, image=red_photo, anchor="w")

    canvas.create_text(600, 300, text="BUTTONS", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 330, text="Stop", font=font_3, fill="White", anchor="e")
    canvas.create_image(625, 330, image=red_photo, anchor="w")
    canvas.create_text(600, 350, text="Button 2", font=font_3, fill="White", anchor="e")
    canvas.create_image(625, 350, image=red_photo, anchor="w")

    canvas.create_text(600, 390, text="SIMA", font=font_1, fill="White", anchor="e")
    canvas.create_text(600, 420, text="Sima 1", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 440, text="Sima 2", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 460, text="Sima 3", font=font_3, fill="White", anchor="e")
    canvas.create_text(600, 480, text="Sima 4", font=font_3, fill="White", anchor="e")

    canvas.create_image(625, 420, image=green_photo, anchor="w")
    canvas.create_image(625, 440, image=red_photo, anchor="w")
    canvas.create_image(625, 460, image=red_photo, anchor="w")
    canvas.create_image(625, 480, image=red_photo, anchor="w")

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((320, 300), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)
    canvas.create_image(665, 150, image=frame_photo, anchor="nw")

    data_text = tk.Text(
    canvas,
    width=48, height=14, font=font_4, wrap=tk.WORD, fg="white", bg="black")
    data_text.config(state='disabled')  # Start as read-only
    canvas.create_window(675, 160, window=data_text, anchor="nw")

    # Start the ROS2 monitor loop
    run_ros2_commands(data_text)
