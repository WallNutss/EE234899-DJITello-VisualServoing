import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ImageOpenCV(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.get_logger().info("Press 'q' to exit the node!")
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Replace with your actual image topic
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Image, '/image_raw_processed',10)
        self.cv_bridge = CvBridge()
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # resize them to 1280x720 original datasheet, becasue the driver put them on 960x720
        # https://stackoverflow.com/questions/23853632/which-kind-of-interpolation-best-for-resizing-image
        cv_image = cv2.resize(cv_image, (1280,720), interpolation=cv2.INTER_LINEAR)

        # convert them back to ros topic message
        ros_image_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # publish them back in new topic
        self.publisher.publish(ros_image_msg)

        
def main(args=None):
    rclpy.init(args=args)
    tello_opencv = ImageOpenCV()
    try:
        rclpy.spin(tello_opencv)
    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Leaving the process node").info("ERROR")
    tello_opencv.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
