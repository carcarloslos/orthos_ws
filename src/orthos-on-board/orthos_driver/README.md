# Drive
Nach dem Starten des Treibers mittels ```roslaunch orthos_driver robot.launch``` werden die Topics ```orthos_velocity_controller_1/command``` (1-6) zur Verfügung gestellt, in die eine Geschwindigkeit in **rad/s** gepublisht werden kann. 

# Real hardware and simulation
Damit der reale Rover mit allen Paketen funktioniert, muss sichergestellt werden, dass folgende Einstellungen in der orthos_description korrekt sind. Andernfalls versuchen die Pakete mit einem simulierten Rover zu arbeiten:

In den Dateien ```orthos_rocker_bogie_left``` und ```orthos_rocker_bogie_right``` müssen folgende Einstellungen getätigt sein:

Der Absatz
```
  <gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>
  </plugin>
</gazebo>
```
darf nur in einer der beiden Dateien vorhanden sein, wenn die Simulation genutzt werden soll. Andernfalls kann dieser einfach gelöscht werden.

Bei allen transmissions muss im HardwareInterface-Tag ```hardwareInterface/VelocityJointInterface``` oder ```hardwareInterface/PositionJointInterface``` stehen. Das ```hardwareInterface/``` darf nur bei Nutzung der realen Hardware in den xacro-Dateien stehen. In der SImulation wird stattdessen das im oberen Absatz "entfernte" ```gazebo_ros_control/DefaultRobotHWSim```-Plugin verwendet.

In dem Paket ```orthos_ackermann``` müssen in der ```orthos_ackerman_node.cpp``` die richtigen Topics eingestellt werden. In Zeile 40 - 43 stehen die Topics für die Lenkmotoren (```orthos_position_controller_left_front/command```). Hier müssen die Topics reingeschrieben werden, auf denen die Motoren die Lenkwinkel empfangen. In Zeile 45 - 50 stehen die für die Antriebsmotoren (```orthos_velocity_controller_1/command```). Hier bitte die Topics einstellen, die von den realen Motoren aufgemacht werden.

# Troubleshooting:
When starting robot.launch and the following error occurs, CAN is not ready. So ROS can not find the hardware Interface to bind to.
```Controller Spawner couldn't find the expected controller_manager ROS interface.```
You can also check that by executing:
```rosservice call /locomotion/driver/init``` after starting robot.launch


# Useful hyperlinks

[CanOpen for ROS](http://wiki.ros.org/ros_canopen)

[Controller configuration](http://wiki.ros.org/canopen_motor_node)

[Fairly complex examples](https://github.com/ipa320/schunk_robots/blob/indigo_dev/schunk_lwa4d/launch/robot.launch)

[Example configuration](https://answers.ros.org/question/313745/how-to-use-controller_manager-with-ros_canopen/)

[Useful step-by-step guide](https://github.com/ros-industrial/ros_canopen/issues/283)

[Second useful step-by-step guide especially for MAXON motors](https://answers.ros.org/question/250174/how-to-control-maxon-motor-by-using-ros_canopen/)

