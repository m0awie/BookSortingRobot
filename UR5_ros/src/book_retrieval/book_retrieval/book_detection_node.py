import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from book_retrieval.msg import BookInfo  # Import the custom message

class BookDetectionNode(Node):
    def __init__(self):
        super().__init__('book_detection_node')
        self.image_subscriber = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.book_publisher = self.create_publisher(BookInfo, 'book_info', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw detected markers
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

        # Detect books and calculate width
        book_info = self.detect_books(image)
        if book_info:
            self.book_publisher.publish(book_info)
            self.get_logger().info(f"Book position and width published: {book_info}")

        # Display the processed image for debugging
        cv2.imshow("Image", image)
        cv2.waitKey(1)

    def detect_books(self, image):
        # Color segmentation to find the book
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([0, 0, 0])  # Modify based on book colors
        upper_bound = np.array([180, 255, 80])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Calculate width based on bounding box of contour
                x, y, w, h = cv2.boundingRect(c)
                width = w  # Approximate width in pixels

               
                book_info = BookInfo()
                book_info.position.x = cX
                book_info.position.y = cY
                book_info.position.z = 0  # Adjust with real depth if available
                book_info.width = float(width)

                return book_info
        return None

def main(args=None):
    rclpy.init(args=args)
    node = BookDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

