# Projeto 3 de Programação de Sistemas Robóticos 2022/23

Trabalho realizado por:
- Diogo Monteiro
- Filipe Gonçalves
- Guilherme Cajeira
- Rafael Oliveira


# Instalação

- Instalar turtlebot:
```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```

- Exportar modelo do turtlebot:

    Abrir .bashrc do editor de escolha, p.e.: `gedit ~/.bashrc`

    Acrescentar esta linha no fim e guardar: `export TURTLEBOT3_MODEL=waffle_pi`

- Clonar repositórios do turtlebot simulation:
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

- Instalar dependências python:
```bash
cd robutler_perception
pip install -r requirements.txt
```

- Clonar dependências e modelos necessários:
```bash
cd ~/catkin_ws/src/
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world
git clone https://github.com/ros-perception/vision_msgs.git
git clone https://github.com/oguran/models
```

- No fim correr catkin_make:
```bash
cd ~/catkin_ws && catkin_make
```

# Correr simulação
Com os packages do repositório do projeto no catkin_ws/src:

## Iniciar mundo:
Num terminal: `roslaunch robutler_bringup gazebo.launch`


## Iniciar robô:
Noutro terminal: `roslaunch robutler_bringup bringup.launch`

É gerado o teleop, que é controlado pelas teclas 'a','w','d','s','q' e space key.


## Para a navegação:
Noutro terminal: `roslaunch robutler_navigation localization.launch`

Noutro terminal: `roslaunch robutler_navigation move_base.launch`

Para indicar a pose inicial e o goal de navegação a partir do Rviz, selecionar primeiro o 'map' como fixed frame nas global settings.

## Para o spawn de objetos na casa:
Noutro terminal: `rosrun psr_apartment_description spawn_object.py`

    Se não especificar qual o objeto e local, o spawn é random

Para especificar o objeto e local do spawn (exemplos): 
`rosrun psr_apartment_description spawn_object.py _place:=<place> _object:=<object>`

`rosrun psr_apartment_description spawn_object.py _place:=<place> _object:=-1`
  
`rosrun psr_apartment_description spawn_object.py _place:=-1 _object:=<object>`

`rosrun psr_apartment_description spawn_object.py _place:=-1 _object:=-1`

## Para o menu interativo:
Noutro terminal: `rosrun robutler_menu menu.py`

No Rviz: adicionar um display de 'Interactive Markers' e escolher o tópico '/menu/update'

Aparecerá por cima do robô uma esfera vermelha, basta fazer clique direito nela para interagir.

De modo a que o robô execute as missões é necessário noutro terminal correr o task_vizualizer da seguinte forma: `rosrun psr_apartment_description task_visualizer.py`.

Depois de o task_visualizer estar a correr, é so selecionar no menu as missóes que pretende executar.

Também aparecerá um teleop, iniciado pelo ficheiro `teleop/src/key_teleop.py`, em que pode mexer as teclas w/s para aumentar/diminuir a velocidade linear, a/d para aumentar/diminuir a velocidade angular, space-bar para parar por completo o robo e q para terminar a sessão.

# Missões
- Look for refrigerator in kitchen
- Look for bottle in dining_area
- Look for book in bedroom
- Look for stop_sign outside
- Look for person in gym
- Count chair in the dining_area
- Look for laptop in office
- Take photo of office

# Nota sobre função de agarrar
O branch `imp/claw` contém uma funcionalidade adicional de abrir e fechar uma garra que permitiria interagir com objectos, no entanto a sua implementação não está terminada e permanece então separada do branch principal, pelo que tem requisitos adicionais de instalação e poderá interferir com as outras funcionalidades.
