import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk


# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

green_photo = None
red_photo = None
green_square_photo = None

empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

empty_checkbox_image = Image.open(empty_checkbox_path).resize((40, 40), Image.LANCZOS)
selected_checkbox_image = Image.open(selected_checkbox_path).resize((40, 40), Image.LANCZOS)

checkbox_states = {"Strategy1": False, "Strategy2": False}

def update_checkbox_image(canvas, selected_id, other_id):
    for checkbox_id in [selected_id, other_id]:
        found_items = canvas.find_withtag(checkbox_id)
        if not found_items:
            print(f"Error: No checkbox found with tag {checkbox_id}")
            continue  # Skip updating if not found

        image_to_display = ImageTk.PhotoImage(selected_checkbox_image if checkbox_states[checkbox_id] else empty_checkbox_image)
        canvas.itemconfig(found_items[0], image=image_to_display)
        canvas.image_cache[checkbox_id] = image_to_display  # Prevent garbage collection
def toggle_checkbox(selected_id, canvas):
    other_id = "Strategy1" if selected_id == "Strategy2" else "Strategy2"
    checkbox_states[selected_id] = True
    checkbox_states[other_id] = False
    update_checkbox_image(canvas, selected_id, other_id)
    selector_var.set(selected_id)

def create_checkbox(canvas, x, y, selected_id, selector_text):
    empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    selected_photo = ImageTk.PhotoImage(selected_checkbox_image)
    canvas.image_cache[selected_id] = empty_photo  # Prevent garbage collection

    canvas.create_image(x, y, image=empty_photo, anchor="center", tags=selected_id)
    canvas.create_text(x + 30, y + 5, text=selector_text, font=tkFont.Font(family="Courier", size=32), fill="White", anchor="w")
    canvas.tag_bind(selected_id, "<Button-1>", lambda e: toggle_checkbox(selected_id, canvas))

def on_selector_change(selected_option):
    toggle_checkbox(selected_option, canvas)

other_checkbox_states = {"set0": False}

def other_update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    existing_image = canvas.find_withtag(servo_id)
    if other_checkbox_states[servo_id]:
        image_to_display = checkbox_selected_photo
    else:
        image_to_display = checkbox_empty_photo

    # If the image already exists, just update it
    if existing_image:
        canvas.itemconfig(existing_image[0], image=image_to_display)
    else:
        canvas.create_image(x, y, image=image_to_display, anchor="center", tags=servo_id)

def other_toggle_checkbox(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo):
    other_checkbox_states[servo_id] = not other_checkbox_states[servo_id]
    other_update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)

def other_create_checkbox(canvas, x, y, servo_id):
    # Load images for this specific checkbox (instead of using global variables)
    empty_checkbox_image = Image.open(empty_checkbox_path)
    selected_checkbox_image = Image.open(selected_checkbox_path)
    empty_checkbox_image = empty_checkbox_image.resize((40, 40), Image.LANCZOS)
    selected_checkbox_image = selected_checkbox_image.resize((40, 40), Image.LANCZOS)

    checkbox_empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    checkbox_selected_photo = ImageTk.PhotoImage(selected_checkbox_image)

    # Set initial checkbox image (unchecked state)
    other_update_checkbox_image(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo)

    canvas.create_text(x+30, y+5, text="Set everything to 0", font=tkFont.Font(family="Courier", size=32), fill="White", anchor="w")
    # Bind the click event to toggle the checkbox state
    canvas.tag_bind(servo_id, "<Button-1>", lambda e: other_toggle_checkbox(servo_id, x, y, canvas, checkbox_empty_photo, checkbox_selected_photo))




def start_competition(canvas):
    from frames.main_frame import switch_frame
    from frames.competition_frame import competition_frame

    switch_frame(competition_frame)  # Change to the competition frame

    if hasattr(canvas, "countdown"):
        canvas.countdown(100)  # Start countdown with 100 seconds

def strategy_frame(canvas):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global button_photo, selector_var
    canvas.image_cache = {}


    font_1 = tkFont.Font(family="Courier", size=42)
    font_2 = tkFont.Font(family="Courier", size=38)
    font_3 = tkFont.Font(family="Courier", size=32)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="INITIALIZE", font=font_title, fill="White", anchor="center")
   
    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((330, 50), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    options = ["Strategy1", "Strategy2"]
    default_value = "Strategy1"

    selector_var = tk.StringVar()
    selector_var.set(default_value)

    canvas.create_text(500, 175, text="Strategy", font=font_2, fill="White", anchor="w")

    create_checkbox(canvas, 515, 210, "Strategy1", "Strategy 1")
    create_checkbox(canvas, 515, 260, "Strategy2", "Strategy 2")

    canvas.create_text(500, 350, text="Motors & Servos", font=font_2, fill="White", anchor="w")
    other_create_checkbox(canvas, 515, 390, "set0")

    img_initialize = canvas.create_image(500, 470, image=button_photo, anchor="w")
    txt_initialize = canvas.create_text(665, 474, text="Start Robot", font=font_2, fill="White", anchor="center")

    canvas.tag_bind(img_initialize, "<Button-1>", lambda e: start_competition(canvas))
    canvas.tag_bind(txt_initialize, "<Button-1>", lambda e: start_competition(canvas))
        

    toggle_checkbox(default_value, canvas)

    #TO-DO: PROGRAM THE FUNCTIONALITY OF THE STRATEGIES (probably send the strategy via ross)
    '''
    canvas.tag_bind(img_strat1, "<Button-1>", lambda e: )
    canvas.tag_bind(txt_strat1, "<Button-1>", lambda e: )

    canvas.tag_bind(img_strat2, "<Button-1>", lambda e: )
    canvas.tag_bind(txt_strat2, "<Button-1>", lambda e: )
    '''

