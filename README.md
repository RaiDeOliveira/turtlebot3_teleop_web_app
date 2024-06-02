# Aplicação web para teleoperação do Turtlebot3

Esse repositório armazena:

1. Uma aplicação web para teleoperação o Turtlebot3 Burger. A aplicação web consiste em uma única página com *transmissão de vídeo capturado por uma câmera acoplada ao Turtlebot3*, *latência da transmissão de vídeo* e *botões para movimentação do robô*. [Clique aqui para ver um vídeo de demonstração](https://youtu.be/91uqqaluQJc).

2. Um pacote ROS2 com um script de CLI para controle da movimentação do Turtlebot3 Burger. A CLI permite que o usuário controle a movimentação do robô por meio das teclas de setas de seu teclado, bem como que o usuário visualize a velocidade do robô em cada eixo em tempo real. [Clique aqui para ver um vídeo de demonstração](https://youtu.be/0zj6vrRGgRs).

# Aplicação web

## Funcionamento

A aplicação web se baseia em Flask e possui uma única página. Essa página, que pode ser encontrada em `~/web-app/templates/index.html`, se trata de um template armazenado em formato HTML com script em Javascript já embutido.

Essa página possui um elemento que é a **transmissão de vídeo em tempo real**, obtida através de uma comunicação via Rosbridge/Websocket com o Turtlebot3. As imagens do vídeo capturado pela câmera são enviadas e recebidas através do tópico ROS `/video_stream`.

Além da transmissão de vídeo, a página apresenta, em sua parte inferior, a latência do recebimento de cada frame do vídeo em milissegundos (ms). Essa informação é enviada e recebida através do tópico ROS `/video_latency`.

Por fim, há os botões de movimentação do Turtlebot3, que enviam comandos ao tópico `/cmd_vel`. Existem 4 botões: um para fazer o robô se mover para frente, um para fazê-lo se mover para trás, um para fazê-lo virar à esquerda e um para fazê-lo virar à esquerda. Cada um desses botões é marcado por uma seta que indica a direção de movimentação a qual o botão se refere.

## Inicialização

### Pré-requisitos

1. Git instalado e com chave SSH configurada
2. Sistema operacional Linux (Ubuntu 22.04 recomendado)
3. ROS2 instalado
4. Python instalado
5. Flask instalado
6. Pacote para execução do Rosbridge instalado
7. Robô Turtlebot3 Burger conectado na mesma rede wi-fi que seu computador (ou simulador Webots)

### Passo a passo

1. Com seu robô Turtlebot3 Burger com ROS2 e pacotes do Turtlebot3 instalados, rode o seguinte comando numa janela de terminal aberta nele (localmente ou via SSH):

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

> :bulb: **Com simulador Webots:** em uma janela de terminal em seu próprio computador, execute o seguinte comando para executar o Webots com um projeto pré-existente contendo o Turtlebot3 Burger: `ros2 launch webots_ros2_turtlebot robot_launch.py`.

2. Ainda no seu robô, abra outra janela de terminal e digite os seguintes comandos para clonar o repositório e executar o script responsável pela transmissão de vídeo e latência:

```bash
git clone git@github.com:RaiDeOliveira/turtlebot3_teleop_web_app.git
cd turtlebot3_teleop_web_app/web-app
python3 sender.py
```

> :bulb: **Com simulador Webots:** em uma outra janela de terminal em seu próprio computador, com o repositório já clonado, execute o seguinte comando para executar o mesmo script e transmitir a imagem de sua webcam: `python3 turtlebot3_teleop_web_app/web-app/sender.py`.

3. Ainda no seu robô, abra outra janela de terminal e execute a comunicação via Rosbridge através do seguinte comando:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> :bulb: **Com simulador Webots:** em uma outra janela de terminal em seu próprio computador, execute o mesmo comando: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`.

4. Em seu computador, abra uma janela de terminal e clone o repositório com o seguinte comando:

```bash
git clone git@github.com:RaiDeOliveira/turtlebot3_teleop_web_app.git
```

4. Na mesma janela de terminal, adentre o diretório da aplicação web e inicie aplicação web através dos seguinte comandos:

```bash
cd turtlebot3_teleop_web_app/web-app
flask --app app run --debug
```

5. Acesse o link que será exibido na janela de terminal após o último comando para utilizar a aplicação web em seu navegador.


# CLI

## Funcionamento

O script responsável pela execução dessa CLI é baseado em Python e usa biblioteca `rclpy` para criar um nó `turtlebot3_teleop` com:

- um **pubblisher**, que publica dados de velocidade para o tópico `cmd_vel` para controlar a velocidade do Turtlebot3 Burger em cada eixo e, portanto, sua movimentação;
- um **subscriber**, que se subscreve ao tópico `cmd_vel` para receber os dados de velocidade atual do robô e exibí-los ao usuário por meio do terminal.

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
git clone git@github.com:RaiDeOliveira/turtlebot3_teleop_web_app.git
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
