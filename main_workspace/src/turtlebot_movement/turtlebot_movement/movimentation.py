import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys, select, os
import tty, termios, time

if os.name == 'nt':
    import msvcrt

BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.5

msg = """
CLI para controle da movimentação do robô
---------------------------
Controles:
       ↑
  ←    ↓    →
↑ : mova para frente
←/→ : mova para a direita/esquerda
↓ : pare de andar
Q : botão de emergência (interromper o programa)
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.key_pressed = None
        self.last_key_pressed = None
        self.running = True  # To control thread lifecycle
        self.lock = threading.Lock()
        self.mensagem = True

    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:  # Check if still supposed to run
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    with self.lock:
                        key = sys.stdin.read(1)
                        if key == '\x1b':  # Esc key
                            key += sys.stdin.read(2)  # Read the next two bytes
                        self.key_pressed = key
                else:
                    with self.lock:
                        self.key_pressed = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    def run(self):
        target_linear_vel = 0.0
        target_angular_vel = 0.0
        key_thread = threading.Thread(target=self.key_poll)
        key_thread.start()

        try:
            print(msg)
            while rclpy.ok():
                with self.lock:
                    key = self.key_pressed
                    last_key = self.last_key_pressed

                if key == 'q':
                    print("INTERROMPENDO MOVIMENTAÇÃO DO ROBÔ")
                    self.running = False  # Signal thread to stop
                    break  # Exit the loop to stop the node

                # Reset speeds if necessary when switching keys
                if key != last_key:
                    if key in ['\x1b[A', '\x1b[B', None]:  # Arrow up, Arrow down
                        target_angular_vel = 0.0
                    if key in ['\x1b[C', '\x1b[D', '\x1b[B', None]:  # Arrow right, Arrow left, Arrow down
                        target_linear_vel = 0.0

                if key == '\x1b[A':  # Arrow up
                    if self.mensagem:
                        print("O robô está andando para frente")
                    self.mensagem = False
                    target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                elif key == '\x1b[D':  # Arrow left
                    if self.mensagem:
                        print("O robô está virando para esquerda")
                    self.mensagem = False
                    target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
                elif key == '\x1b[C':  # Arrow right
                    if self.mensagem:
                        print("O robô está virando para direita")
                    self.mensagem = False
                    target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                elif key == '\x1b[B' or key is None:  # Arrow down
                    if not self.mensagem:
                        print("O robô está parando")
                    self.mensagem = True
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0

                self.last_key_pressed = key  # Update the last pressed key

                twist = Twist()
                twist.linear.x = float(target_linear_vel)
                twist.angular.z = float(target_angular_vel)
                self.publisher_.publish(twist)
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.running = False
            key_thread.join()

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
