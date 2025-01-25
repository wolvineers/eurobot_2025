import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import cv2
import os

os.system("clear")
current_directory = os.path.dirname(os.path.abspath(__file__))

# Set global variables
cap = None
update_running = False
camera_open = True

#Set the function to open the camera
def camera_frame(canvas, width=460, height=380):
    global cap, update_running, frame_photo, camera_open

    #Get data to resize frame.png
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    _ret, _frame = cap.read()
    _frame = cv2.cvtColor(_frame, cv2.COLOR_BGR2RGB)
    _img = Image.fromarray(_frame)
    _imgW, _imgH = _img.size
    _imgRatio = _imgW/_imgH
    _imgBaseSize = 450

    frame_path = os.path.join(current_directory, "../img/frame.png")
    frame_image = Image.open(frame_path)
    frame_image = frame_image.resize((int(_imgBaseSize*_imgRatio), _imgBaseSize), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    canvas.create_image(465, 75, image=frame_photo, anchor="nw")

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
            imgBaseSize = 400
            img = img.resize((int(imgBaseSize*imgRatio), imgBaseSize))
            imgtk = ImageTk.PhotoImage(image=img)
            canvas.create_image(500, 100, anchor=tk.NW, image=imgtk)
            canvas.image = imgtk
        canvas.after(10, update_frame)

    update_frame()

#Set the function to close the camera
def close_camera():
    global update_running, cap
    if update_running:
        update_running = False 
        if cap is not None:
            cap.release() 
        cv2.destroyAllWindows()  
        print("Cámara cerrada.")
