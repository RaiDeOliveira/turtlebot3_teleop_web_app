import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
from time import time

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        # Cria os publishers para os tópicos
        self.img_publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.latency_publisher = self.create_publisher(String, '/video_latency', 10)
        # Configura a captura de vídeo da webcam
        self.timer = self.create_timer(0.01, self.timer_callback)  # Frequência de 100 Hz
        self.cap = cv2.VideoCapture(0)  # Captura de vídeo da webcam padrão
        self.latency = time()  # Inicializa o tempo de latência

    def timer_callback(self):
        # Lê um frame da webcam
        ret, frame = self.cap.read()
        if ret:
            # Codifica o frame como JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            # Cria a mensagem de imagem comprimida
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            # Publica a mensagem no tópico '/video_frames'
            self.img_publisher_.publish(msg)
        # Calcula a latência e publica no tópico '/video_latency'
        latency_string = str(((int(round(time() - self.latency, 4) * 1000))))
        latency_msg = String()
        latency_msg.data = latency_string
        self.latency_publisher.publish(latency_msg)
        # Atualiza o tempo de latência
        self.latency = time()
    
def main(args=None):
    rclpy.init(args=args)  # Inicializa o sistema ROS
    webcam_publisher = WebcamPublisher()  # Cria uma instância do nó
    rclpy.spin(webcam_publisher)  # Mantém o nó em execução
    webcam_publisher.destroy_node()  # Finaliza o nó
    rclpy.shutdown()  # Finaliza o sistema ROS

if __name__ == '__main__':
    main()  # Chama a função principal
