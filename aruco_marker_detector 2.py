import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco
import numpy as np
import pyrealsense2 as rs
import math
import pytesseract

class ArucoMarkerDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.bridge = CvBridge()
        
        # Define ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Subscribe to color and depth images
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)

        # Publisher for movement commands "lookup here"
        self.move_pub = self.create_publisher(String, 'move_commands', 10)

        # Subscriber to listen for movement completion "lookup here"
        self.move_done_sub = self.create_subscription(String, 'move_done', self.move_done_callback, 10)
        
        # Storage for latest color and depth images
        self.latest_color_image = None
        self.latest_depth_image = None
        self.z_plane = None  # Z-coordinate for the ArUco markers to align book centroids

        # Status for receiving movement completion signal
        self.move_done = False

    def color_callback(self, msg):
        # Convert color image to OpenCV format
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.latest_depth_image is not None:
            self.detect_books()

    def depth_callback(self, msg):
        # Convert depth image to OpenCV format
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def move_done_callback(self, msg):
        if msg.data == "done":
            self.move_done = True
            self.get_logger().info("Move completed, ready for next command.")

    def detect_books(self):
    # Detect ArUco markers to define shelf boundaries and z-plane
    corners, ids, _ = cv2.aruco.detectMarkers(self.latest_color_image, self.aruco_dict, parameters=self.aruco_params)
    if ids is not None and len(ids) >= 2:
        # Assume markers at both ends of the shelf are detected
        self.get_logger().info("Detected markers at the ends of the shelf.")
        left_marker = self.calculate_marker_position(corners[0])
        right_marker = self.calculate_marker_position(corners[-1])

        if left_marker is None or right_marker is None:
            return  # Abort if markers are not valid

        # Set z-plane based on the z-coordinate of the markers
        self.z_plane = left_marker[2]
        self.get_logger().info(f"Setting z-plane for books to {self.z_plane} meters")

        # Define middle row to scan for gaps along the x-axis
        middle_y = int((left_marker[1] + right_marker[1]) / 2)
        row = middle_y
        prev_depth = self.latest_depth_image[row, 0]
        book_start_x = None
        book_centroids = []
        
        # Scan across the row from left to right marker
        for col in range(int(left_marker[0]), int(right_marker[0])):
            depth = self.latest_depth_image[row, col]

            # Detect gap indicating the end of a book
            if book_start_x is not None and abs(depth - prev_depth) > 50:
                # Calculate x, y centroid and set z to match the z-plane
                book_x = (book_start_x + col) // 2
                book_y = row
                book_centroid = [book_x, book_y, self.z_plane]
                book_centroids.append(book_centroid)

                # Crop region for OCR and extract text
                book_roi = self.latest_color_image[book_y-10:book_y+10, book_x-10:book_x+10]
                title = self.extract_text(book_roi)
                
                # Print the book title and centroid to the terminal
                print(f"Detected book title: {title} at position {book_centroid}")
                self.get_logger().info(f"Detected book title: {title} at position {book_centroid}")

                book_centroids.append((title, book_centroid))
                book_start_x = None
            elif abs(depth - prev_depth) <= 50:
                if book_start_x is None:
                    book_start_x = col
            prev_depth = depth

        # Sort and process the books based on the centroids
        sorted_books = sorted(book_centroids, key=lambda book: book[0])
        self.sort_books(sorted_books)


    def extract_text(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY_INV)
        text = pytesseract.image_to_string(thresh_image, config="--psm 6")
        return text.strip()

    def sort_books(self, sorted_books):
        temp_left = [-0.2, 0.0, self.z_plane]
        temp_right = [0.2, 0.0, self.z_plane]

        for index, (title, target_centroid) in enumerate(sorted_books):
            current_centroid = self.get_current_position(title)  # Placeholder for current book position

            if current_centroid != target_centroid:
                temp_slot = temp_left if index % 2 == 0 else temp_right
                self.send_move_command(current_centroid, temp_slot, target_centroid)
                while not self.move_done:
                    rclpy.spin_once(self)
                self.move_done = False

        self.send_move_command("all_done")

    def get_current_position(self, title):
        # Placeholder function: Replace with actual method to retrieve current book positions
        # For example, a dictionary of titles and their initial centroids.
        return [0, 0, self.z_plane]  # Replace with actual current position

    def send_move_command(self, centroid_1, centroid_2=None, centroid_3=None):
        if centroid_1 == "all_done":
            command = "all_done"
        else:
            command = f"{centroid_1};{centroid_2};{centroid_3}"
        
        self.get_logger().info(f"Sending move command: {command}")
        self.move_pub.publish(String(data=command))

    def calculate_marker_position(self, corner):
        marker_size = 0.05
        obj_points = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)
        
        img_points = np.array(corner[0], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.get_intrinsics(), None)
        
        if success:
            return self.transform_to_base(tvec, rvec)
        return None

    def get_intrinsics(self):
        intrinsics = np.array([[600.0, 0, 320.0], [0, 600.0, 240.0], [0, 0, 1]])
        return intrinsics
    
    def transform_to_base(self, tvec, rvec):
        T_cam_to_base = self.get_camera_to_base_transform()
        tvec_hom = np.array([tvec[0], tvec[1], tvec[2], 1])
        tvec_base = T_cam_to_base @ tvec_hom
        return tvec_base[:3]

    def get_camera_to_base_transform(self):
        cam_x, cam_y, cam_z = 0.14798, -0.31868, 0.22643
        roll, pitch, yaw = math.radians(-176.84), math.radians(-0.24), math.radians(91.69)
        R = self.rotation_matrix_from_rpy(roll, pitch, yaw)
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = [cam_x, cam_y, cam_z]
        return T

    def rotation_matrix_from_rpy(self, roll, pitch, yaw):
        R_x = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        R_y = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        return R_z @ R_y @ R_x

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoMarkerDetectorNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

