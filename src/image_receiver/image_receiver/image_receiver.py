import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import time
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header


class ImageReceiver(Node):
    def __init__(self):
        self.bridge = CvBridge()
        self.last_receive_time = time.time()
        self.framce_count = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()

        self.current_image = None
        self.image_timestamp = None

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.compressed_subscription = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/compressed",
            self.compressed_image_callback,
            image_qos,
        )
        self.raw_subscription = self.create_subscription(
            CompressedImage,
            "/camera/image_raw/raw",
            self.compressed_image_callback,
            image_qos,
        )

        self.processed_image_publisher = self.create_publisher(
            Image,
            "/host/processed_image",
            image_qos,
        )
        self.slam_image_publisher = self.create_publisher(
            Image,
            "/host/slam/image",
            image_qos,
        )

        self.stats_timer = self.create_timer(2.0, self.print_statistics)

        self.declare_parameter("enable_display", True)
        self.declare_parameter("image_scale", 1.0)
        self.declare_parameter("enable_slam_preprocessing", True)
        self.declare_parameter("target_fps", 15)

        self.get_logger().info("Image Receiver Node initialized")
        self.get_logger().info("Waiting for images from Raspberry Pi...")
        self.get_logger().info("Subscribed to:")
        self.get_logger().info("  - /camera/image_raw/compressed")
        self.get_logger().info("  - /camera/image_raw")

    def compressed_image_callback(self, payload: CompressedImage):
        try:
            np_arr = np.frombuffer(payload.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image.")
                return
            self.process_image(cv_image, payload.header)

        except Exception as e:
            self.get_logger().error(f" Error processing compressed image: {str(e)}")

    def raw_image_callback(self, payload: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(payload, desired_encoding="bgr8")
            self.process_image(cv_image, payload.header)
        except Exception as e:
            self.get_logger().error(f" Error processing compressed image: {str(e)}")

    def process_image(self, cv_image, header: Header):
        current_time = time.time()
        self.frame_count += 1
        self.fps_counter += 1
        self.last_receive_time = current_time

        self.current_image = cv_image.copy()
        self.image_timestamp = header.stamp

        enable_display = self.get_parameter("enable_display").value
        image_scale = self.get_parameter("image_scale").value
        enable_slam_prep = self.get_parameter("enable_slam_preprocessing").value

        if image_scale != 1.0:
            height, width = cv_image.shape[:2]
            new_width = int(width * image_scale)
            new_height = int(height * image_scale)
            cv_image = cv2.resize(cv_image, (new_width, new_height))

        processed_image = cv_image.copy()

        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            processed_msg.header = header
            processed_msg.header.frame_id = "camera_optical_frame"
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing processed image: {str(e)}")

        if enable_slam_prep:
            slam_image = self.prepare_for_slam(cv_image)
            try:
                slam_msg = self.bridge.cv2_to_imgmsg(slam_image, encoding="mono8")
                slam_msg.header = header
                slam_msg.header.frame_id = "camera_optical_frame"
                self.slam_image_pub.publish(slam_msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing SLAM image: {str(e)}")

        # Display image locally if enabled (for debugging)
        if enable_display:
            self.display_image(processed_image)

    def prepare_for_slam(self, image):
        # gray = cv2.cvtColor(image, cv2.ColorBGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize(8,8))
        # image = clahe.apply(gray)
        return image

    def display_image(self, image):
        try:
            cv2.imshow("Received Image", image)
            cv2.waitKey(1)

        except Exception as e:
            pass

    def print_statistics(self):
        current_time = time.time()
        time_since_last = current_time - self.last_receive_time

        if time_since_last > 5.0:
            self.get_logger().warn(
                f"No images received for {time_since_last:.1f} seconds"
            )
            self.get_logger().warn("Check network connection to Raspberry Pi")
        else:
            fps = getattr(self, "current_fps", 0)
            self.get_logger().info(
                f"Image Stats - FPS: {fps:.1f}, Total Frames: {self.frame_count}, "
                f"Last received: {time_since_last:.1f}s ago"
            )

    def get_latest_image(self):
        return self.current_image, self.image_timestamp

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        image_receiver = ImageReceiver()

        import signal

        def signal_handler(signum, frame):
            image_receiver.get_logger().info("Shutting down image receiver...")
            image_receiver.destroy_node()
            rclpy.shutdown()

        signal.signal(signal.SIGINT, signal_handler)

        rclpy.spin(image_receiver)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in image receiver: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
