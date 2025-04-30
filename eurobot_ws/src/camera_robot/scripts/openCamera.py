import cv2

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Usamos CAP_V4L2 para cámaras en Linux/Docker

    if not cap.isOpened():
        print("❌ Error: No se puede abrir la cámara.")
        return
    print("✅ Cámara abierta correctamente.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error: No se pudo capturar la imagen.")
            break

        cv2.imshow('Vista de la cámara', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("👋 Cerrando la ventana...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

