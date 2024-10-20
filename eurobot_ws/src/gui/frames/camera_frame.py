import tkinter as tk
import tkinter.font as tkFont
from PIL import Image, ImageTk
import cv2
import os

os.system("clear")

# Set global variables
cap = None
update_running = False
camera_open = True

#Set the function to open the camera
def camera_frame(canvas, width=460, height=380):
    global cap, update_running, frame_photo, camera_open

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    frame_image = Image.open("eurobot_ws/src/gui/img/frame.png")
    frame_image = frame_image.resize((460, 290), Image.LANCZOS)
    frame_photo = ImageTk.PhotoImage(frame_image)

    canvas.create_image(530, 145, image=frame_photo, anchor="nw")

    update_running = True 
    print("Camera ON")

    def update_frame():
        if not update_running:
            return
        ret, frame = cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            imgtk = ImageTk.PhotoImage(image=img)
            canvas.create_image(550, 170, anchor=tk.NW, image=imgtk)
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
