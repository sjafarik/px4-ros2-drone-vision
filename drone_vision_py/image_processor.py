import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        self.declare_parameter(
            'image_topic',
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter('display_original', False)
        self.declare_parameter('display_processed', True)
        self.declare_parameter('log_every_n_frames', 30)
        self.declare_parameter('blur_kernel_size', 5)
        self.declare_parameter('canny_threshold1', 80)
        self.declare_parameter('canny_threshold2', 150)

        self.image_topic = self.get_parameter('image_topic').value
        self.display_original = self.get_parameter('display_original').value
        self.display_processed = self.get_parameter('display_processed').value
        self.log_every_n_frames = self.get_parameter('log_every_n_frames').value
        self.blur_kernel_size = self.get_parameter('blur_kernel_size').value
        self.canny_threshold1 = self.get_parameter('canny_threshold1').value
        self.canny_threshold2 = self.get_parameter('canny_threshold2').value

        if self.blur_kernel_size % 2 == 0:
            self.blur_kernel_size += 1
            self.get_logger().warn(
                f'blur_kernel_size must be odd, adjusted to {self.blur_kernel_size}'
            )

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
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        self.frame_count += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(
            gray,
            (self.blur_kernel_size, self.blur_kernel_size),
            0
        )
        edges = cv2.Canny(
            blurred,
            self.canny_threshold1,
            self.canny_threshold2
        )

        if self.frame_count % self.log_every_n_frames == 0:
            height, width = frame.shape[:2]
            self.get_logger().info(
                f'Processed frame {self.frame_count}: width={width}, height={height}'
            )

        if self.display_original:
            resized_original = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow('Drone Camera Original', resized_original)

        if self.display_processed:
            resized_edges = cv2.resize(edges, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow('Drone Camera Edges', resized_edges)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()

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