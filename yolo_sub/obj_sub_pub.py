import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from ultralytics import YOLO


class YoloPoseSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_pose_subscriber')

        # Subscription to the compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Replace with your topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for the modified image
        self.publisher = self.create_publisher(
            CompressedImage,
            '/image_raw/detections/compressed',
            10)

        # Load the YOLO model
        self.model = YOLO('yolov8n.pt')  # Load YOLO model
        self.get_logger().info('YOLO Pose Subscriber initialized.')

    def listener_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Run YOLO pose estimation
            results = self.model(cv_image)

            # Overlay detections on the image
            for result in results:
                boxes = result.boxes  # Detections bounding boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Get coordinates
                    confidence = box.conf[0]  # Get confidence
                    label = box.cls[0]  # Get class label
                    class_name = self.model.names[int(label)]  # Map class ID to name

                    # Draw rectangle and label on the image
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'{class_name} {confidence:.2f}',
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Encode the image back to compressed format
            _, encoded_image = cv2.imencode('.jpg', cv_image)
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header  # Copy the original message's header
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(encoded_image).tobytes()

            # Publish the modified image
            self.publisher.publish(compressed_msg)
            self.get_logger().info('Published image with detections.')

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
