# 🚗 ROS 2 – Gazebo Bridge Tutorial

## 1. List Gazebo Topics
First, check what topics Gazebo is publishing:

```bash
gz topic -l
```

👉 Look for:
- **Odometry** topic → e.g. `/model/vehicle_blue/odometry`
- **Velocity command** topic → e.g. `/cmd_vel`

---

## 2. Create the Bridges
Use `ros_gz_bridge` to connect Gazebo and ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge   /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 3. Check ROS 2 Topics
Now verify that the topics are visible in ROS 2:

```bash
ros2 topic list
```

You should see:
- `/model/vehicle_blue/odometry`
- `/cmd_vel`

---

## 4. Subscribe to Odometry
To view odometry data from the robot:

```bash
ros2 topic echo /model/vehicle_blue/odometry
```

---

## 5. Drive the Robot
Send velocity commands from ROS 2 to Gazebo:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}" -r 10
```

- `linear.x = 0.3` → forward speed (m/s)  
- `angular.z = 0.1` → turning rate (rad/s)  

---

✅ That’s it! You can now drive your Gazebo robot using ROS 2 and monitor its odometry.
