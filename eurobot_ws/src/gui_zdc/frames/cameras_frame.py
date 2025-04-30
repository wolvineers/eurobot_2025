import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
from frames.camera_1_frame import camera_1_frame
from frames.camera_2_frame import camera_2_frame
import cv2
import os

# Set the global variables
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set global variables
cap = None
update_running = False
camera_open = True

def cameras_frame(canvas, width=460, height=380):
    """ 
    Set the function to design the frame00 (Control Systems Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    global cap, update_running, frame_photo, camera_open, button_photo

    #Get data to resize frame.png
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    _ret, _frame = cap.read()
    _frame = cv2.cvtColor(_frame, cv2.COLOR_BGR2RGB)
    _img = Image.fromarray(_frame)
    _imgW, _imgH = _img.size
    _imgRatio = _imgW/_imgH
    _imgBaseSize = 200

    # Set the variable that contains the path of the image
    button_path = os.path.join(current_directory, "../img/white-button.png")
    button_image = Image.open(button_path)
    button_image = button_image.resize((265, 45), Image.LANCZOS)
    button_photo = ImageTk.PhotoImage(button_image)

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((int(_imgBaseSize*_imgRatio), _imgBaseSize), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)
    camera_1_canvas = canvas.create_image(600, 250, image=frame_photo, anchor="center")
    camera_2_canvas = canvas.create_image(900, 250, image=frame_photo, anchor="center")

    update_running = True 
    print("Camera ON")

    def update_frame():
        if not update_running:
            return
        ret, frame = cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgW, imgH = img.size
            imgRatio = imgW/imgH
            imgBaseSize = 175
            img = img.resize((int(imgBaseSize*imgRatio), imgBaseSize))
            imgtk = ImageTk.PhotoImage(image=img)
            camera_1_canvas = canvas.create_image(600, 250, anchor="center", image=imgtk)
            camera_2_canvas = canvas.create_image(900, 250, anchor="center", image=imgtk)
            canvas.image = imgtk
        canvas.after(10, update_frame)

    update_frame()

    font_1 = tkFont.Font(family="Courier", size=24)
    font_2 = tkFont.Font(family="Courier", size=20)
    font_3 = tkFont.Font(family="Courier", size=16)
    font_4 = tkFont.Font(family="Courier", size=14)
    font_title = tkFont.Font(family="Courier", size=64)
    numbers_big = tkFont.Font(family="Orbitron", size=126)

    canvas.create_text(750, 100, text="CAMERAS", font=font_title, fill="White", anchor="center")

    img_calibrate = canvas.create_image(750, 500, image=button_photo, anchor="center")
    txt_calibrate = canvas.create_text(750, 500, text="Calibrate", font=font_1, fill="White", anchor="center")

    camera_1_btn = canvas.create_image(600, 400, image=button_photo, anchor="center")
    camera_1_txt = canvas.create_text(600, 400, text="ZdC", font=font_1, fill="White", anchor="center")

    camera_2_btn = canvas.create_image(900, 400, image=button_photo, anchor="center")
    camera_2_txt = canvas.create_text(900, 400, text="Robot", font=font_1, fill="White", anchor="center")

    canvas.tag_bind(camera_1_btn, "<Button-1>", lambda e: switch_frame_camera(camera_1_frame))
    canvas.tag_bind(camera_1_txt, "<Button-1>", lambda e: switch_frame_camera(camera_1_frame))

    canvas.tag_bind(camera_2_btn, "<Button-1>", lambda e: switch_frame_camera(camera_2_frame))
    canvas.tag_bind(camera_1_txt, "<Button-1>", lambda e: switch_frame_camera(camera_2_frame))

def switch_frame_camera(frame_function):
    global update_running, cap, canvas
    from frames.main_frame import switch_frame
    if update_running:
        update_running = False 
        if cap is not None:
            cap.release() 
        cv2.destroyAllWindows()  
        print("Cámara cerrada.")
    
    switch_frame(frame_function)

def close_camera():
    global update_running, cap
    if update_running:
        update_running = False 
        if cap is not None:
            cap.release() 
        cv2.destroyAllWindows()  
        print("Cámara cerrada.")
