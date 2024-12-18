import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO


class YoloPoseSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pose_subscriber')

        # Initialize subscriber
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Replace with your topic name
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Load YOLO model once
        self.model = YOLO('yolov8n-pose.pt')  
        self.get_logger().info('YOLO Pose Subscriber initialized and model loaded.')

    def listener_callback(self, msg):
        try:
            # Decode the compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Run YOLO inference with the correct format
            self.model.predict(source=cv_image, show=True, conf=0.25, stream=False)

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
