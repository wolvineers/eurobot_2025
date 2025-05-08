import tkinter.font as tkFont
import os
from PIL import Image, ImageTk
import subprocess
import threading
import time

from frames.gui_node import get_node  # Acceder al nodo ROS 2

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
frame_photo = None
sensor_icons = {}  # Diccionario para mapear sensores y botones a imágenes

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

def update_sensor_icons(canvas, gui_node):
    sensor_states = gui_node.get_sensor_states()
    button_states = gui_node.get_button_states()

    for name, icon_id in sensor_icons.items():
        if name.startswith("Sensors"):
            state = sensor_states.get(name, 0)
        elif name.startswith("Button"):
            state = button_states.get(name, 0)
        else:
            state = 0
        image = green_photo if state == 1 else red_photo
        canvas.itemconfig(icon_id, image=image)

    canvas.after(5000, lambda: update_sensor_icons(canvas, gui_node))

def panel_frame(canvas):
    global green_photo, red_photo, frame_photo, sensor_icons
    from frames.main_frame import tk

    # Fonts
    font_1 = tkFont.Font(family="Courier", size=32)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=10)
    font_title = tkFont.Font(family="Courier", size=64)

    # Load icons
    green_image = Image.open(os.path.join(current_directory, "../img/green.png")).resize((16, 16), Image.LANCZOS)
    green_photo = ImageTk.PhotoImage(green_image)

    red_image = Image.open(os.path.join(current_directory, "../img/red.png")).resize((16, 16), Image.LANCZOS)
    red_photo = ImageTk.PhotoImage(red_image)

    # Static UI
    canvas.create_text(750, 100, text="PANEL", font=font_title, fill="White", anchor="center")
    canvas.create_text(615, 170, text="SENSORS", font=font_1, fill="White", anchor="e")

    # 5 sensores
    sensor_labels = ["Sensors1", "Sensors2", "Sensors3", "Sensors4", "Sensors5"]
    for i, name in enumerate(sensor_labels):
        canvas.create_text(615, 200 + 20 * i, text=name, font=font_3, fill="White", anchor="e")
        icon_id = canvas.create_image(640, 200 + 20 * i, image=red_photo, anchor="w")
        sensor_icons[name] = icon_id

    # 2 botones
    canvas.create_text(615, 320, text="BUTTONS", font=font_1, fill="White", anchor="e")
    button_labels = ["Button1", "Button2"]
    for i, name in enumerate(button_labels):
        canvas.create_text(615, 350 + 20 * i, text=name, font=font_3, fill="White", anchor="e")
        icon_id = canvas.create_image(640, 350 + 20 * i, image=red_photo, anchor="w")
        sensor_icons[name] = icon_id

    # Frame image
    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path).resize((300, 275), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)
    canvas.create_image(760, 150, image=frame_photo, anchor="nw")

    # ROS2 diagnostics output box
    data_text = tk.Text(canvas, width=33, height=14, font=font_4, wrap=tk.WORD, fg="white", bg="black")
    data_text.config(state='disabled')
    canvas.create_window(775, 165, window=data_text, anchor="nw")
    run_ros2_commands(data_text)

    # Start update loop
    ros_node = get_node()
    update_sensor_icons(canvas, ros_node)
