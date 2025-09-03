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
- **Wheel state** topic → `/world/<world_name>/model/<robot_name>/joint_state`

---

## 2. Create the Bridges
Use `ros_gz_bridge` to connect Gazebo and ROS 2:

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

**Frame:** Robot base frame (`base_link`).

**Example command:**
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.3}, angular: {z: 0.1}}" -r 10
```

---

## 5. Odometry (`/model/<robot_name>/odometry`)
- **Gazebo type:** `gz.msgs.Odometry`  
- **ROS 2 type:** `nav_msgs/msg/Odometry`

### Contains:
- **Twist:** robot’s instantaneous velocity (in `base_link`).  
- **Pose:** robot’s position (x, y, z) and orientation (quaternion) in the **world frame**.  
  - Pose ≈ integrated twist over time.  
  - Errors accumulate in real robots; simulation is exact unless noise added.

**View in ROS 2:**
```bash
ros2 topic echo /model/vehicle_blue/odometry
```

---

## 6. Wheel State (Encoders / Joint State)
Topic (Gazebo):
```
/world/<world_name>/model/<robot_name>/joint_state
```
- **Type:** `gz.msgs.Model`  
- Reports **joint positions** (angles), **velocities**, and **forces**.

**Wheel distance traveled:**
```
s = θ × r
```
- `θ` = joint position (radians)  
- `r` = wheel radius (m)

---

## 7. Message Flow Diagram
```
/cmd_vel (Twist command)
        ↓
   Robot plugin
        ↓
Odometry Twist (velocity in base_link)
        ↓  integrate over time
Odometry Pose (position + orientation in world)
        ↓
Wheel State (joint encoders: rotation per wheel)
```

---

✅ **Summary**
- **Velocity command (`/cmd_vel`)**: tells the robot how fast to move.  
- **Odometry Twist**: robot’s actual velocity.  
- **Odometry Pose**: integrated position/orientation in world.  
- **Wheel state**: encoder/joint info (how much each wheel turned).  
- With the bridge running, you can command the robot in ROS 2 and monitor its motion from Gazebo.  
