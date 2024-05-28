import rclpy  # Importa a biblioteca ROS 2 para Python
from rclpy.node import Node  # Importa a classe Node da biblioteca ROS 2
from geometry_msgs.msg import Twist  # Importa a mensagem Twist para comandos de velocidade
import threading  # Importa a biblioteca de threads
import sys, select, os  # Importa bibliotecas do sistema
import tty, termios, time  # Importa bibliotecas para manipulação do terminal e tempo

# Verifica se o sistema operacional é Windows e importa msvcrt se for o caso
if os.name == 'nt':
    import msvcrt

# Definição de constantes para velocidades máximas e incrementos de velocidade
BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.5

# Mensagem de instruções para controle do robô
msg = """
Controle da movimentação do robô
---------------------------
Controles:

       ↑
  ←    ↓    →

↑ : move para frente
← : move para a esquerda
→ : move para a direita
↓ : pare de andar
Q : botão de emergência (interromper o programa)
"""

# Define a classe Teleop que herda da classe Node
class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')  # Inicializa o nó com o nome 'turtlebot3_teleop'
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)  # Cria um publisher para o tópico 'cmd_vel'
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)  # Cria um subscriber para o tópico 'cmd_vel'
        self.key_pressed = None  # Variável para armazenar a tecla pressionada
        self.last_key_pressed = None  # Variável para armazenar a última tecla pressionada
        self.running = True  # Variável para controlar o ciclo de vida da thread
        self.lock = threading.Lock()  # Cria um lock para sincronização de threads
        self.mensagem = True  # Variável para controle de mensagens
        self.current_twist = Twist()  # Inicializa a mensagem Twist

    # Callback para o subscriber que atualiza a mensagem Twist atual
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: linear_x={msg.linear.x}, angular_z={msg.angular.z}')
        self.current_twist = msg

    # Função para capturar a tecla pressionada
    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:  # Verifica se ainda deve continuar rodando
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    with self.lock:
                        key = sys.stdin.read(1)
                        if key == '\x1b':  # Tecla Esc
                            key += sys.stdin.read(2)  # Lê os próximos dois bytes
                        self.key_pressed = key
                else:
                    with self.lock:
                        self.key_pressed = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    # Função para imprimir as velocidades atuais a cada 1 segundo
    def print_velocities(self):
        while self.running:
            with self.lock:
                print(f'Velocidades atuais: linear_x={self.current_twist.linear.x}, angular_z={self.current_twist.angular.z}')
            time.sleep(1)

    # Função principal de controle do robô
    def run(self):
        target_linear_vel = 0.0  # Velocidade linear alvo inicial
        target_angular_vel = 0.0  # Velocidade angular alvo inicial
        key_thread = threading.Thread(target=self.key_poll)  # Cria a thread para captura de teclas
        print_thread = threading.Thread(target=self.print_velocities)  # Cria a thread para imprimir velocidades
        key_thread.start()
        print_thread.start()

        try:
            print(msg)
            while rclpy.ok():  # Loop principal enquanto ROS está ok
                with self.lock:
                    key = self.key_pressed
                    last_key = self.last_key_pressed

                if key == 'q':  # Botão de emergência
                    print("INTERROMPENDO MOVIMENTAÇÃO DO ROBÔ")
                    self.running = False  # Sinaliza para parar as threads
                    break  # Sai do loop principal

                # Reseta as velocidades se necessário ao trocar de tecla
                if key != last_key:
                    if key in ['\x1b[A', '\x1b[B', None]:  # Seta para cima, Seta para baixo
                        target_angular_vel = 0.0
                    if key in ['\x1b[C', '\x1b[D', '\x1b[B', None]:  # Seta para direita, Seta para esquerda, Seta para baixo
                        target_linear_vel = 0.0

                # Ajusta as velocidades com base na tecla pressionada
                if key == '\x1b[A':  # Seta para cima
                    if self.mensagem:
                        print("O robô está andando para frente")
                    self.mensagem = False
                    target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                
                elif key == '\x1b[D':  # Seta para esquerda
                    if self.mensagem:
                        print("O robô está virando para esquerda")
                    self.mensagem = False
                    target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
              
                elif key == '\x1b[C':  # Seta para direita
                    if self.mensagem:
                        print("O robô está virando para direita")
                    self.mensagem = False
                    target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                
                elif key == '\x1b[B' or key is None:  # Seta para baixo
                    if not self.mensagem:
                        print("O robô está parando")
                    self.mensagem = True
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0

                self.last_key_pressed = key  # Atualiza a última tecla pressionada

                # Cria a mensagem Twist com as velocidades calculadas
                twist = Twist()
                twist.linear.x = float(target_linear_vel)
                twist.angular.z = float(target_angular_vel)
                self.publisher_.publish(twist)  # Publica a mensagem Twist
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        
        finally:
            # Para o robô ao finalizar
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.running = False
            key_thread.join()  # Espera a thread de captura de teclas finalizar
            print_thread.join()  # Espera a thread de impressão de velocidades finalizar

# Função principal para inicializar e rodar o nó
def main(args=None):
    rclpy.init(args=args)  # Inicializa o sistema ROS
    teleop = Teleop()  # Cria uma instância da classe Teleop
    teleop.run()  # Executa a função run
    rclpy.shutdown()  # Encerra o sistema ROS

if __name__ == '__main__':
    main()  # Executa a função principal se o script for executado diretamente
