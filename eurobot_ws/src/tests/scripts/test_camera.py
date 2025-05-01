import cv2

cap = cv2.VideoCapture(0)  # Usa la cámara por defecto (video0)

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Vista de la cámara', frame)

    if cv2.waitKey(1) == ord('q'):  # Presiona 'q' para salir
        break

cap.release()
cv2.destroyAllWindows()
