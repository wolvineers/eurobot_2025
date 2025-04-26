import cv2
import numpy as np

def euclidean_distance(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def group_aruco_markers(centers, max_distance=150):
    groups = []
    visited = set()

    for i, center in enumerate(centers):
        if i in visited:
            continue
        group = [center]
        visited.add(i)

        for j, other_center in enumerate(centers):
            if j not in visited and euclidean_distance(center, other_center) < max_distance:
                group.append(other_center)
                visited.add(j)

        groups.append(group)
    return groups

def main():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: No se puede abrir la cámara.")
        return

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict)

    target_area_px2 = 40000  # Área objetivo para detenerse

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: No se pudo capturar la imagen.")
            break

        height, width, _ = frame.shape
        image_center_x = width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        detected_centers = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            max_area = 0  # Inicializa área máxima

            for marker_corners in corners:
                corner_array = marker_corners[0]
                # Centroides
                cx = int(np.mean(corner_array[:, 0]))
                cy = int(np.mean(corner_array[:, 1]))
                detected_centers.append((cx, cy))
                cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

                # Área
                area = cv2.contourArea(corner_array)
                if area > max_area:
                    max_area = area

            # Agrupación
            groups = group_aruco_markers(detected_centers, max_distance=150)
            cans = []

            for group in groups:
                if len(group) == 2 or len(group) == 1:
                    cans.append(group)

            # Dibuja rectángulo exterior
            all_points = np.concatenate(corners, axis=1).reshape(-1, 2).astype(np.int32)
            rect = cv2.minAreaRect(all_points)
            box = cv2.boxPoints(rect)
            box = box.astype(np.int32)
            cv2.drawContours(frame, [box], 0, (0, 255, 255), 2)

            # Centrando latas
            if len(cans) >= 1:
                all_centers = []
                for group in cans:
                    xs = [p[0] for p in group]
                    all_centers.append(np.mean(xs))

                avg_center_x = int(np.mean(all_centers))
                cv2.circle(frame, (avg_center_x, height // 2), 10, (0, 0, 255), -1)

                x_margin = 20

                if abs(avg_center_x - image_center_x) > x_margin:
                    direction = "derecha" if avg_center_x < image_center_x else "izquierda"
                    print(f"➡ Mover robot hacia {direction} para centrar las latas.")
                else:
                    print("✅ Latas centradas. Listo para avanzar recto.")
            else:
                print("No se detectaron latas claramente.")

            # Movimiento hacia delante si no estamos lo suficientemente cerca
            if max_area < target_area_px2:
                print(f"🔄 Área máxima detectada: {int(max_area)} px^2. Avanzando hacia la lata...")
                # Aquí deberías controlar el movimiento real del robot
                # robot.move_forward()
            else:
                print(f"🛑 Área objetivo alcanzada: {int(max_area)} px^2. ¡Detente!")
                # robot.stop()

            print(f"Detectadas {len(cans)} latas.")
        else:
            print("No se detectaron arucos.")

        # Línea central
        cv2.line(frame, (image_center_x, 0), (image_center_x, height), (255, 255, 0), 2)
        cv2.imshow('Detección de Latas con Arucos', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
