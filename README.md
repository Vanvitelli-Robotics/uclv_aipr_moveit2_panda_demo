# Panda MoveIt! demo

[![N|Solid](https://www.unicampania.it/doc/img/logo_vanvitelli.jpg)](https://www.ingegneria.unicampania.it/roboticslab)

Demo rete ros con planning MoveIt2! per il corso di Programmazione dei Robot.

## Install

- Prerequisiti: Installare Moveit!

Nella cartella src del ros workspace
```bash
git clone https://github.com/Vanvitelli-Robotics/moveit_resources_panda_moveit_config.git
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

## Costruire la scena


In un nuovo terminale avviare il simulatore:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

In un nuovo terminale avviare il server di generazione scena:
```bash
ros2 run uclv_aipr_moveit2_panda_demo scene_builder
```

In un nuovo terminale chiamare il servizio generazione scena.
Ci sono due possibilità

1- Costruire tutta la scena compresa di oggetto da afferrare
```bash
ros2 service call /build_scene std_srvs/srv/SetBool "{data: True}"
```

2- Costruire solo la scena **escluso** di oggetto da afferrare
```bash
ros2 service call /build_scene std_srvs/srv/SetBool "{data: False}"
```
Nel secondo caso dovrete creare voi l'oggetto afferrare nel vostro codice

## Avviare la demo

Avviare il simulatore e costruire la scena chiamando il servizio con "data:True" (punto 1).
Avviare il task:
```bash
ros2 run uclv_aipr_moveit2_panda_demo demo_moveit
```
Ogni volta che il terminale si ferma su:
```bash
Press a key to continue...
```
Inserire un qualsiasi **singolo** carattere e premere invio per continuare.

## Codice
Il codice dei due nodi è disponibile in questo repo.

## License

GNU General Public License v3.0