import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO


class YoloPoseSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pose_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Replace with your topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.model = YOLO('yolov8n.pt')  # Load YOLO model
        self.get_logger().info('YOLO Pose Subscriber initialized.')

    def listener_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Run YOLO pose estimation
            self.model(source=cv_image, show=True)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
