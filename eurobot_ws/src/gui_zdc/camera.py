import cv2

def main():
    try:
        cam_index = int(input("Introduce el número de la cámara (por ejemplo, 0, 1, 2...): "))
    except ValueError:
        print("Índice inválido.")
        return

    cap = cv2.VideoCapture(cam_index)

    if not cap.isOpened():
        print(f"No se pudo abrir la cámara con índice {cam_index}")
        return

    print("Presiona 'q' para salir")

    cv2.namedWindow("Cámara", cv2.WINDOW_NORMAL)  # Asegura UNA ventana

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el frame")
            break

        cv2.imshow("Cámara", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
