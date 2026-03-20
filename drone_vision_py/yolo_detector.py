import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter(
            'image_topic',
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('resize_scale', 0.5)
        self.declare_parameter('log_every_n_frames', 30)
        self.declare_parameter('show_window', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.resize_scale = self.get_parameter('resize_scale').value
        self.log_every_n_frames = self.get_parameter('log_every_n_frames').value
        self.show_window = self.get_parameter('show_window').value

        self.bridge = CvBridge()
        self.frame_count = 0
        self.is_processing = False

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)
        self.get_logger().info('YOLO model loaded successfully')

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')

    def image_callback(self, msg: Image):
        if self.is_processing:
            return

        self.is_processing = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            self.is_processing = False
            return

        self.frame_count += 1

        try:
            results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            self.is_processing = False
            return

        annotated_frame = results[0].plot()

        boxes = results[0].boxes
        detection_count = 0 if boxes is None else len(boxes)

        if self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: detected {detection_count} object(s)'
            )

        if self.show_window:
            resized = cv2.resize(
                annotated_frame,
                (0, 0),
                fx=self.resize_scale,
                fy=self.resize_scale
            )
            cv2.imshow('YOLO Detection View', resized)
            cv2.waitKey(1)

        self.is_processing = False


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()

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