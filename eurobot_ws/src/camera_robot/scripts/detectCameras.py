import cv2

def find_available_cameras(max_index=10):
    available_cameras = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"? Cámara disponible en el índice {i}")
            available_cameras.append(i)
            cap.release()
        else:
            print(f"? No se encontró cámara en el índice {i}")
    return available_cameras

if __name__ == "__main__":
    cameras = find_available_cameras()
    if cameras:
        print(f"Cámaras activas encontradas: {cameras}")
    else:
        print("No se encontraron cámaras activas.")
