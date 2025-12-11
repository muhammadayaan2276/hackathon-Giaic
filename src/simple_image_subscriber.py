import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SimpleImageSubscriber(Node):

    def __init__(self):
        super().__init__('simple_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.get_logger().info('Image subscriber node started.')

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # Display image - uncomment for local testing with GUI
        # cv2.imshow("camera", current_frame)
        # cv2.waitKey(1)

        # In a real application, you would process the image here
        # e.g., apply object detection, feature extraction, etc.
        # For this example, we just acknowledge receipt.

def main(args=None):
    rclpy.init(args=args)
    node = SimpleImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
