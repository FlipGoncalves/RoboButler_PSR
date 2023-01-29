# Installation
Partindo do principio que já têm o catkin_ws configurado:

install turtlebot:
```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
```

exportar modelo do turtlebot:

abrir .bashrc: `gedit ~/.bashrc`

por esta linha no fim e guardar: `export TURTLEBOT3_MODEL=waffle_pi`

clone and build turtlebot simulation repositories:
```bash
cd ~/catkin_ws/src/
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

download worlds:
```bash
cd ~/catkin_ws/src
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world
```

# Run simulation
Com os packages do repositório do projeto no catkin_ws/src:

Iniciar mundo:
Num terminal: `roslaunch robutler_bringup gazebo.launch`

Iniciar robo:
Noutro terminal: `roslaunch robutler_bringup bringup.launch`

Para a navigation:
Noutro terminal: `roslaunch robutler_navigation localization.launch`
(após esse correr) Noutro terminal: `roslaunch robutler_navigation move_base.launch`

# Tasks

TODO:
- Ajustar configuração do robo (customizar cor/extra thingies como antena p.e.)
- Criar teleop melhorado (movimento manual)
- Mapeamento e navegação
- Spawn de objectos (começar com objetos simples para avançar para perceção)
- Perceção (requer spawn)
	- simples: detetar objectos de cor solida
	- mais avançada: pessoas e objetos com modelos neuronais já treinados

## Notas:
- condução com informação semantica -> apenas um menu interativo no rviz tipo "go to"->"kitchen/bedroom"
- detetar pessoas/objetos complexos -> procurar redes neuronais já treinadas para isso
- spawn de objectos deve ter posição randomized dentro da divisão especificada
- don't bother with advanced missions
- iniciar missões é feito com menu interativo rviz
- inventar missões novas para além das descritas
- construir mapa uma vez só com SLAM, guardar (ros map server)

