import tkinter.font as tkFont
import tkinter as tk
import os
from PIL import Image, ImageTk
from frames.second_initialize_frame import second_initialize_frame


# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

empty_checkbox_path = os.path.join(current_directory, "../img/checkbox_empty.png")
selected_checkbox_path = os.path.join(current_directory, "../img/checkbox_selected.png")

empty_checkbox_image = Image.open(empty_checkbox_path).resize((40, 40), Image.LANCZOS)
selected_checkbox_image = Image.open(selected_checkbox_path).resize((40, 40), Image.LANCZOS)

# Define available strategies
strategies = ["GrocNavegacio", "GrocManual", "BlauNavegacio", "BlauManual"]
checkbox_states = {strategy: False for strategy in strategies}

# Update the checkbox image based on selection
def update_checkbox_image(canvas):
    for strategy in strategies:
        found_items = canvas.find_withtag(strategy)
        if not found_items:
            print(f"Error: No checkbox found with tag {strategy}")
            continue
        
        image_to_display = ImageTk.PhotoImage(
            selected_checkbox_image if checkbox_states[strategy] else empty_checkbox_image
        )
        canvas.itemconfig(found_items[0], image=image_to_display)
        canvas.image_cache[strategy] = image_to_display  # Prevent garbage collection

# Toggle the selected strategy
def toggle_checkbox(selected_id, canvas):
    for strategy in strategies:
        checkbox_states[strategy] = (strategy == selected_id)
    update_checkbox_image(canvas)
    selector_var.set(selected_id)

# Create a checkbox for a strategy
def create_checkbox(canvas, x, y, strategy_id, strategy_text):
    empty_photo = ImageTk.PhotoImage(empty_checkbox_image)
    canvas.image_cache[strategy_id] = empty_photo  # Prevent garbage collection

    canvas.create_image(x, y, image=empty_photo, anchor="center", tags=strategy_id)
    canvas.create_text(x + 30, y + 5, text=strategy_text, font=tkFont.Font(family="Courier", size=32), fill="White", anchor="w")
    canvas.tag_bind(strategy_id, "<Button-1>", lambda e: toggle_checkbox(strategy_id, canvas))

# Start the competition
def start_competition(canvas):
    from frames.main_frame import switch_frame
    from frames.fourth_initialize_frame import fourth_initialize_frame
    switch_frame(fourth_initialize_frame)

# Initialize the frame
def third_initialize_frame(canvas):
    global button_photo, selector_var, back_photo, background_photo, forward_photo
    canvas.image_cache = {}

    from frames.main_frame import switch_frame, window

    #Set the shape of the window
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()

    font_title = tkFont.Font(family="Courier", size=54)
    font_2 = tkFont.Font(family="Courier", size=38)

    back_path = os.path.join(current_directory, "../img/icons8-back-24.png")
    back_image = Image.open(back_path)
    back_image = back_image.resize((48, 48), Image.LANCZOS)
    back_photo = ImageTk.PhotoImage(back_image)


    background_path = os.path.join(current_directory, "../img/background.jpg")
    background_image = Image.open(background_path)
    background_image = background_image.resize((int(screen_width), int(screen_height)), Image.LANCZOS)
    background_photo = ImageTk.PhotoImage(background_image)

    canvas.create_image(0, 0, image=background_photo, anchor="nw")
    img_back = canvas.create_image(24, 24, image=back_photo, anchor="nw")
    canvas.tag_bind(img_back, "<Button-1>", lambda e: switch_frame(second_initialize_frame))

    forward_photo = ImageTk.PhotoImage(back_image.rotate(180))
    img_forward = canvas.create_image(1128, 24, image=forward_photo, anchor="ne")
    canvas.tag_bind(img_forward, "<Button-1>", lambda e: start_competition(canvas))


    canvas.create_text(600, 50, text="● ● ● ○", font=font_title, fill="White", anchor="center")
    canvas.create_text(600, 125, text="STRATEGY", font=font_title, fill="White", anchor="center")



    selector_var = tk.StringVar()
    selector_var.set(strategies[0])

    y_position = 210
    for strategy in strategies:
        create_checkbox(canvas, 475, y_position, strategy, strategy)
        y_position += 50
    
    toggle_checkbox(strategies[0], canvas)
