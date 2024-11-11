import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import cv2.aruco
import pyrealsense2 as rs
import numpy as np
import math
import pytesseract

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # Define ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # Subscribe to color and depth images
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        
        # Publisher for book centroids and text
        self.book_pub = self.create_publisher(String, 'book_centroids', 10)
        
        # Storage for latest color and depth images
        self.latest_color_image = None
        self.latest_depth_image = None
    
    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.latest_depth_image is not None:
            self.detect_books()
    
    def detect_books(self):
        # Initialize lists to store book centroids and titles
        book_centroids = []
        book_titles = []

        # Define parameters for depth separation threshold (distance between books)
        depth_threshold = 50  # mm; adjust as needed

        # Scan along a horizontal line (e.g., middle row) in the depth image
        row = self.latest_depth_image.shape[0] // 2
        prev_depth = self.latest_depth_image[row, 0]
        book_start_x = 0

        for col in range(1, self.latest_depth_image.shape[1]):
            depth = self.latest_depth_image[row, col]
            if abs(depth - prev_depth) > depth_threshold:
                # End of book detected; calculate centroid of this book region
                book_x = (book_start_x + col) // 2
                book_y = row
                depth_in_meters = depth * 0.001

                # Calculate 3D position
                point_3d = rs.rs2_deproject_pixel_to_point(self.get_intrinsics(), [book_x, book_y], depth_in_meters)
                point_in_base = self.transform_point_to_base(point_3d)

                # Store centroid position
                book_centroids.append(point_in_base)
                self.get_logger().info(f"Book centroid at: x={point_in_base[0]:.2f}, y={point_in_base[1]:.2f}, z={point_in_base[2]:.2f}")

                # Extract text from book spine in color image
                book_roi = self.latest_color_image[:, book_start_x:col]
                book_title = self.extract_text(book_roi)
                book_titles.append(book_title)
                self.get_logger().info(f"Detected book title: {book_title}")

                # Publish information about the detected book
                position_msg = f"Book Title: {book_title}, Position: x={point_in_base[0]:.2f}, y={point_in_base[1]:.2f}, z={point_in_base[2]:.2f}"
                self.book_pub.publish(String(data=position_msg))

                # Reset book start position
                book_start_x = col
            
            prev_depth = depth

    def extract_text(self, image):
        # Preprocess the image for OCR (e.g., grayscale, thresholding)
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Use Tesseract to perform OCR on the processed image
        text = pytesseract.image_to_string(thresh_image, config="--psm 6")  # PSM 6 is good for blocks of text
        return text.strip()

    def get_intrinsics(self):
        intrinsics = rs.intrinsics()
        intrinsics.width = 640
        intrinsics.height = 480
        intrinsics.ppx = 320.0
        intrinsics.ppy = 240.0
        intrinsics.fx = 600.0
        intrinsics.fy = 600.0
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0, 0, 0, 0, 0]
        return intrinsics

    def rotation_matrix_from_rpy(self, roll, pitch, yaw):
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        R = R_z @ R_y @ R_x
        return R

    def get_camera_to_base_transform(self):
        cam_x, cam_y, cam_z = 147.98/1000, -318.68/1000, 226.43/1000  # Position in meters
        roll, pitch, yaw = math.radians(-176.84), math.radians(-0.24), math.radians(91.69)
        R = self.rotation_matrix_from_rpy(roll, pitch, yaw)
        T = np.identity(4)
        T[:3, :3] = R
        T[:3, 3] = [cam_x, cam_y, cam_z]
        return T

    def transform_point_to_base(self, point_in_camera_frame):
        T = self.get_camera_to_base_transform()
        point_in_camera_hom = np.array([point_in_camera_frame[0], point_in_camera_frame[1], point_in_camera_frame[2], 1])
        point_in_base_hom = T @ point_in_camera_hom
        point_in_base = point_in_base_hom[:3]
        return point_in_base

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetectorNode()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

