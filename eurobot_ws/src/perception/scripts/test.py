import cv2

cap = cv2.VideoCapture(4, cv2.CAP_V4L2)  # cambia 4 por el número correcto si es necesario
if not cap.isOpened():
    print("❌ No se puede abrir la cámara")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ No se pudo leer frame")
        break
    cv2.imshow("Camara", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
