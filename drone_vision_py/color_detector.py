import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')

        self.declare_parameter(
            'image_topic',
            '/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image'
        )
        self.declare_parameter('resize_scale', 0.5)
        self.declare_parameter('log_every_n_frames', 30)
        self.declare_parameter('min_contour_area', 300)

        # Blue default HSV range
        self.declare_parameter('lower_h', 100)
        self.declare_parameter('lower_s', 150)
        self.declare_parameter('lower_v', 50)
        self.declare_parameter('upper_h', 140)
        self.declare_parameter('upper_s', 255)
        self.declare_parameter('upper_v', 255)

        self.image_topic = self.get_parameter('image_topic').value
        self.resize_scale = self.get_parameter('resize_scale').value
        self.log_every_n_frames = self.get_parameter('log_every_n_frames').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        self.lower_h = self.get_parameter('lower_h').value
        self.lower_s = self.get_parameter('lower_s').value
        self.lower_v = self.get_parameter('lower_v').value
        self.upper_h = self.get_parameter('upper_h').value
        self.upper_s = self.get_parameter('upper_s').value
        self.upper_v = self.get_parameter('upper_v').value

        self.lower_color = np.array([self.lower_h, self.lower_s, self.lower_v])
        self.upper_color = np.array([self.upper_h, self.upper_s, self.upper_v])

        self.bridge = CvBridge()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to image topic: {self.image_topic}')
        self.get_logger().info(
            f'HSV range: lower={self.lower_color.tolist()}, upper={self.upper_color.tolist()}'
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        self.frame_count += 1

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        output = frame.copy()

        image_height, image_width = frame.shape[:2]
        image_center_x = image_width // 2
        image_center_y = image_height // 2

        cv2.circle(output, (image_center_x, image_center_y), 5, (0, 255, 255), -1)

        detected = False

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area >= self.min_contour_area:
                M = cv2.moments(largest_contour)

                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    cv2.drawContours(output, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 6, (0, 0, 255), -1)
                    cv2.line(
                        output,
                        (image_center_x, image_center_y),
                        (cx, cy),
                        (255, 0, 0),
                        2
                    )

                    error_x = cx - image_center_x
                    error_y = cy - image_center_y

                    cv2.putText(
                        output,
                        f'Target: ({cx}, {cy})',
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        output,
                        f'Error: dx={error_x}, dy={error_y}',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    detected = True

                    if self.frame_count % self.log_every_n_frames == 0:
                        self.get_logger().info(
                            f'Detected target at ({cx}, {cy}), area={area:.1f}, '
                            f'error_x={error_x}, error_y={error_y}'
                        )

        if not detected and self.frame_count % self.log_every_n_frames == 0:
            self.get_logger().info('No target detected')

        resized_output = cv2.resize(
            output,
            (0, 0),
            fx=self.resize_scale,
            fy=self.resize_scale
        )

        resized_mask = cv2.resize(
            mask,
            (0, 0),
            fx=self.resize_scale,
            fy=self.resize_scale
        )

        cv2.imshow('Color Detection View', resized_output)
        cv2.imshow('Color Mask', resized_mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()

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