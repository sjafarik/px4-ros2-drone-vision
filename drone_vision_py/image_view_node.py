import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data


class ImageViewNode(Node):
    def __init__(self):
        super().__init__('image_view_node')

        self.declare_parameter(
            'image_topic',
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter('show_image', True)
        self.declare_parameter('log_every_n_frames', 30)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.log_every_n_frames = self.get_parameter('log_every_n_frames').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        self.get_logger().info(f'Image callback triggered: frame {self.frame_count}')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        if self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info(
                f'Received frame {self.frame_count}: width={width}, height={height}'
            )

        if self.show_image:
            resized = cv2.resize(cv_image, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow('Drone Camera View', resized)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()