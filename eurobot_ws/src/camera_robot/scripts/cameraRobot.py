import cv2
import numpy as np
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

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

class ArucoCenterPublisher(Node):
    def __init__(self):
        super().__init__('aruco_center_publisher')
        self.publisher_ = self.create_publisher(Int32, '/object_center', 10)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Error: No se puede abrir la cámara.")
            exit(1)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

        self.target_area_px2 = 40000
        self.position_buffer = []
        self.buffer_start_time = time.time()

        self.timer = self.create_timer(0.01, self.process_frame)  # Cada 10ms

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Error: No se pudo capturar la imagen.")
            return

        height, width, _ = frame.shape
        image_center_x = width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        detected_centers = []

        if ids is not None:
            max_area = 0

            for marker_corners in corners:
                corner_array = marker_corners[0]
                cx = int(np.mean(corner_array[:, 0]))
                cy = int(np.mean(corner_array[:, 1]))
                detected_centers.append((cx, cy))
                area = cv2.contourArea(corner_array)
                if area > max_area:
                    max_area = area

            groups = group_aruco_markers(detected_centers, max_distance=150)
            cans = []

            for group in groups:
                if len(group) == 2 or len(group) == 1:
                    cans.append(group)

            if len(cans) >= 1:
                all_centers = []
                for group in cans:
                    xs = [p[0] for p in group]
                    all_centers.append(np.mean(xs))

                avg_center_x = int(np.mean(all_centers))

                position_percentage = (avg_center_x / width) * 100

                if position_percentage <= 40:
                    position = 0  # Izquierda
                elif position_percentage <= 60:
                    position = 1  # Centro
                else:
                    position = 2  # Derecha

                self.position_buffer.append(position)

        # Si han pasado más de 100ms, procesamos el buffer
        if (time.time() - self.buffer_start_time) >= 0.1:
            if self.position_buffer:
                most_common = max(set(self.position_buffer), key=self.position_buffer.count)
                msg = Int32()
                msg.data = most_common
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publicado: {most_common}")

            self.position_buffer.clear()
            self.buffer_start_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCenterPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
