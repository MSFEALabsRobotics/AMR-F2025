# 🚗 ROS 2 – Gazebo Bridge & Differential Drive Robot Cheat Sheet

This guide explains how to bridge **Gazebo Sim** with **ROS 2**, and details the main messages when controlling a **differential drive robot** with the DiffDrive plugin.

---

## 1. List Gazebo Topics
Check what topics Gazebo is publishing:

```bash
gz topic -l
```

👉 Look for:
- **Odometry** topic → e.g. `/model/vehicle_blue/odometry`
- **Velocity command** topic → e.g. `/cmd_vel`

---

## 2. Create the Bridges
Use `ros_gz_bridge` to connect Gazebo and ROS 2:
The parameter_bridge connects topics between Gazebo (gz) and ROS 2.
- Use the   **"ros2 run ros_gz_bridge parameter_bridge"**    Command
- ros2 run ros_gz_bridge parameter_bridge  <topic_name>@<ros2_msg_type>@<gz_msg_type>


```bash
ros2 run ros_gz_bridge parameter_bridge   /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 3. Check ROS 2 Topics
Verify that topics are visible in ROS 2:

```bash
ros2 topic list
```

Expected:
- `/model/vehicle_blue/odometry`
- `/cmd_vel`

---

## 4. Velocity Command (`/cmd_vel`)
- **Gazebo type:** `gz.msgs.Twist`  
- **ROS 2 type:** `geometry_msgs/msg/Twist`

**Fields (diff drive):**
- `linear.x` → Forward/backward speed (m/s)  
- `angular.z` → Rotation around vertical axis (rad/s)  
- Other components usually `0`

**Frame: Robot Frame.**

**Example command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.3}, angular: {z: 0.1}}" -r 10
```

---

## 5. Odometry (`/model/<robot_name>/odometry`)
- **Gazebo type:** `gz.msgs.Odometry`  
- **ROS 2 type:** `nav_msgs/msg/Odometry`

```bash
nav_msgs/msg/Odometry:
  header:                     # Standard ROS 2 message header
    stamp:                    # Time
      sec: int32
      nanosec: uint32
    frame_id: string          # Reference frame (e.g., "odom")
  child_frame_id: string      # Robot base frame (e.g., "base_link")

  pose:                       # PoseWithCovariance
    pose:                     # geometry_msgs/Pose
      position:               # geometry_msgs/Point
        x: float64            # Position in meters (world frame)
        y: float64
        z: float64
      orientation:            # geometry_msgs/Quaternion
        x: float64            # Orientation (quaternion form)
        y: float64
        z: float64
        w: float64
    covariance: [float64[36]] # 6x6 covariance matrix

  twist:                      # TwistWithCovariance
    twist:                    # geometry_msgs/Twist
      linear:                 # geometry_msgs/Vector3
        x: float64            # Linear velocity in m/s
        y: float64
        z: float64
      angular:                # geometry_msgs/Vector3
        x: float64            # Angular velocity (rad/s)
        y: float64
        z: float64
    covariance: [float64[36]] # 6x6 covariance matrix
```

### Contains:
- a **Header**:
  -frame_id: vehicle_blue/odom → Odometry reference frame.
  -child_frame_id: vehicle_blue/chassis → The robot base frame.
- **Twist:** robot’s instantaneous velocity (in robot frame).  
- **Pose:** robot’s position (x, y, z) and orientation (quaternion) in **world frame**.  
  - Pose ≈ integrated twist over time.  
  - Errors accumulate in real robots; simulation is exact unless noise added.

**View in ROS 2:**
```bash
ros2 topic echo /model/vehicle_blue/odometry
```
---


✅ **Summary**
- **Velocity command (`/cmd_vel`)**: tells the robot how fast to move. (in robot frame)  
- **Odometry Twist**: robot’s actual velocity.  
- **Odometry Pose**: integrated position/orientation in world frame.  
