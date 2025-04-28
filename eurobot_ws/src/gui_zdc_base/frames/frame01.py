import tkinter.font as tkFont
from PIL import Image, ImageTk
import os

os.system("clear")

def get_public_ip():
    try:
        # Use os to run the curl command for public IP
        public_ip = os.popen('curl -s https://api.ipify.org').read().strip()
        return public_ip
    except Exception as e:
        return f"Error: {e}"
    
def get_local_ip():
    try:
        # Use os to run ifconfig with grep and awk to extract local IP
        local_ip = os.popen("ifconfig | grep 'inet ' | grep -v '127.0.0.1' | awk '{print $2}'").read().strip()
        return local_ip
    except Exception as e:
        return f"Error fetching local IP: {e}"


def frame01(canvas):
    """ 
    Set the function to design the frame01 (Settings Frame)

    Args:
        (canvas): Variable that set the shape of the window
    """
    courirer_font = tkFont.Font(family="Courier", size=22) # Set the font for the default text

    # Display public IP (obtained from an online API)
    canvas.create_text(530, 120, text="Public IP", font=courirer_font, fill="White")
    canvas.create_text(900, 120, text=get_public_ip(), font=courirer_font, fill="White")

    #Display local IP (obtained from console)
    canvas.create_text(530, 180, text="Local IP", font=courirer_font, fill="White")
    canvas.create_text(900, 180, text=get_local_ip(), font=courirer_font, fill="White")






    
