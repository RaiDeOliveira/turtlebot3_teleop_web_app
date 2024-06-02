import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
from time import time

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.img_publisher_ = self.create_publisher(
            CompressedImage,
            '/video_frames',
            10
        )
        
        self.latency_publisher = self.create_publisher(
            String,
            '/video_latency',
            10
        )

        self.timer = self.create_timer(0.01, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.cap = cv2.VideoCapture(0)
        self.latency = time()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.img_publisher_.publish(msg)
        latency_string = str(((int(round(time() - self.latency, 4)*1000))))
        latency_msg = String()
        latency_msg.data = latency_string
        self.latency_publisher.publish(latency_msg)
        self.latency = time()
    

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()