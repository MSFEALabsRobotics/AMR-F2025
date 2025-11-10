install nav2

```bash
sudo apt install ros-<ros2-distro>-navigation2
sudo apt install ros-<ros2-distro>-nav2-bringup
```

ros jazzy

to start slam (mapping, and localizing)
```bash
source /opt/ros/<ros2-distro>/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

# Terminal 2 – start SLAM if the demo doesn't do it automatically
```bash
ros2 launch slam_toolbox online_sync_launch.py
```

# Terminal 3 – teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
