# comprobo22warmup


Commands cheat guide:

```
ros2 launch neato_node2 bringup.py host:=192.168.17.207
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Setting up new package:

setup package with:

ros2 pkg create in_class_day03 --build-type ament_python --node-name emergency_stop --dependencies rclpy std_msgs geometry_msgs sensor_msgs neato2_interfaces
--
source install/setup.bash
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
--
Add nodes to setup.py
--
ros2 run [package name] [node name]



Rviz: rviz2
