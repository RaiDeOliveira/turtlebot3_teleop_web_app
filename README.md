# CLI de controle do Turtlebot3 Burger

Esse repositório armazena um pacote ROS2 com um script de CLI para controle da movimentação do Turtlebot3 Burger. A CLI permite que o usuário controle a movimentação do robô por meio das teclas de setas de seu teclado, bem como que o usuário visualize a velocidade do robô em cada eixo em tempo real.

## Funcionamento

O script responsável pela execução dessa CLI é baseado em Python e usa biblioteca `rclpy` para criar um nó `turtlebot3_teleop` com:

- um **pubblisher**, que publica dados de velocidade para o tópico `cmd_vel` para controlar a velocidade do Turtlebot3 Burger em cada eixo e, portanto, sua movimentação;
- um **subscriber**, que se subscreve ao tópico `cmd_vel` para receber os dados de velocidade atual do robô e exibí-los ao usuário por meio do terminal.

## Vídeo de demonstração

Você pode assistir um vídeo de demonstração do projeto clicando [aqui](https://youtu.be/0zj6vrRGgRs).

## Inicialização

### Pré requisitos

1. Git instalado e com chave SSH configurada
2. Sistema operacional Linux (Ubuntu 22.04 recomendado)
3. ROS2 instalado
4. Python instalado
5. Robô Turtlebot3 Burger conectado na mesma rede wi-fi que seu computador (ou simulador Webots)

### Passo a passo

1. Com seu robô Turtlebot3 Burger com ROS2 e pacotes do Turtlebot3 instalados, rode o seguinte comando numa janela de terminal aberta nele (localmente ou via SSH):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

> :bulb: **Com simulador Webots:** em uma janela de terminal em seu próprio computador, execute o seguinte comando para executar o Webots com um projeto pré-existente contendo o Turtlebot3 Burger: `ros2 launch webots_ros2_turtlebot robot_launch.py`.

2. Em seu computador, abra uma janela de terminal e clone o repositório com o seguinte comando:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

3. Adentre o diretório do workspace que contém o pacote ROS2 responsável por inicializar a CLI de controle da movimentação do robô:

```bash
cd /turtlebot3_teleop_web_app/main_workspace
```

4. Execute o seguinte comando para compilar o pacote ROS2:

```bash
colcon build
```

5. Execute o seguinte comando para configurar seu terminal com as variáveis necessárias para executar o script da CLI:

```bash
source install/local_setup.bash
```

6. Execute o seguinte comando para inicializar a CLI:

```bash
ros2 run turtlebot_movement movement_cli
```