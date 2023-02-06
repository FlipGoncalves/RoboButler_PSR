# Installation
Partindo do principio que já têm o catkin_ws configurado:

- Instalar turtlebot:
```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```

- Exportar modelo do turtlebot:

    Abrir .bashrc: `gedit ~/.bashrc`

    Acrescentar esta linha no fim e guardar: `export TURTLEBOT3_MODEL=waffle_pi`

- Clone and build turtlebot simulation repositories:
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

- Instalar dependências python:
```bash
cd robutler_perception
pip install -r requirements.txt
```

- Clone (... faltar escrever):
```bash
cd ~/catkin_ws/src/
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world
git clone https://github.com/ros-perception/vision_msgs.git
git clone https://github.com/oguran/models
cd ~/catkin_ws && catkin_make
```

# Run simulation
Com os packages do repositório do projeto no catkin_ws/src:

## Iniciar mundo:
Num terminal: `roslaunch robutler_bringup gazebo.launch`


## Iniciar robô:
Noutro terminal: `roslaunch robutler_bringup bringup.launch`

É gerado o tele


## Para a navigation:
Noutro terminal: `roslaunch robutler_navigation localization.launch`

Noutro terminal: `roslaunch robutler_navigation move_base.launch`

Para indicar a pose inicial e o goal de navegação a partir do Rviz, selecionar primeiro o 'map' como fixed frame nas global settings

## Para o menu interativo:

Noutro terminal: `rosrun robutler_menu menu.py`

No Rviz: adicionar um display de 'Interactive Markers' e escolher o tópico '/menu/update'

Aparecerá por cima do robô uma esfera vermelha, basta fazer clique direito nela para interagir.

De modo a que o robô execute as missões é necessário noutro terminal correr o task_vizualizer da seguinte forma: `rosrun psr_apartment_description task_visualizer.py`.

Depois de o task_visualizer estar a correr, é so selecionar no menu as missóes que pretende executar.

# Tasks

TODO:
- Criar teleop melhorado (movimento manual)
- Refazer mapa? o atual tem algumas partes estranhas
- Ajustar objetos iniciais, alguns estão fora do sítio
- Continuar script de spawn de objetos
- Menu interativo para navegação
- Missões
- Menu interativo para ativar missões

## Notas:
- condução com informação semantica -> apenas um menu interativo no rviz tipo "go to"->"kitchen/bedroom"
- detetar pessoas/objetos complexos -> procurar redes neuronais já treinadas para isso
- spawn de objectos deve ter posição randomized dentro da divisão especificada
- don't bother with advanced missions
- iniciar missões é feito com menu interativo rviz
- inventar missões novas para além das descritas
- construir mapa uma vez só com SLAM, guardar (ros map server)

