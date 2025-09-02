# 🚗 Differential Drive Robot – Message Cheat Sheet (Gazebo Sim & ROS 2)

This guide explains the main messages when controlling a differential drive robot in **Gazebo Sim** with the **DiffDrive plugin**.

---

## 1. Velocity Command (`/cmd_vel`)
- **Topic name:** `/cmd_vel`
- **Message type (Gazebo):** `gz.msgs.Twist`
- **Message type (ROS 2):** `geometry_msgs/msg/Twist`

**Fields:**
- `linear.x` → Forward/backward speed (m/s)
- `linear.y, linear.z` → Usually **0** for diff drive
- `angular.z` → Rotation around vertical axis (rad/s)
- `angular.x, angular.y` → Usually **0** for diff drive

**Frame:**  
- Expressed in the **robot base frame** (`base_link`).

---

## 2. Odometry (`/model/<robot_name>/odometry`)
- **Message type (Gazebo):** `gz.msgs.Odometry`
- **Message type (ROS 2):** `nav_msgs/msg/Odometry`

### Odometry contains:

#### a. `twist`
- Instantaneous velocity of the robot
- Same structure as `Twist` above  
- **Frame:** Robot base frame (`base_link`)

#### b. `pose`
- Robot’s **estimated position and orientation**  
- Position: (x, y, z) in the **world/inertial frame**  
- Orientation: quaternion (x, y, z, w) relative to the world  

**Relationship:**  
- `pose` ≈ **integrated values of twist over time**  
- Errors accumulate in real robots (slip, encoder noise), but in simulation it is exact unless noise is added.

---

## 3. Wheel State (Encoders / Joint State)
To know how much each wheel has turned:

- **Topic name (Gazebo):**  
  `/world/<world_name>/model/<robot_name>/joint_state`
- **Message type:** `gz.msgs.Model`  
  (contains joint positions, velocities, and forces)

Each wheel joint reports:
- **position** → angle (rad), cumulative rotation  
- **velocity** → angular speed (rad/s)

**Distance traveled by a wheel:**
```
s = θ × r
```
where:
- `θ` = wheel angle (radians, from joint position)  
- `r` = wheel radius (meters)  

---

## 4. Message Flow Diagram

```
/cmd_vel (Twist command)
        ↓
   Robot plugin
        ↓
Twist (measured: current velocity in base_link)
        ↓  integrate over time
Pose (position + orientation in world frame)
```

---

✅ Summary:
- **Velocity command (`cmd_vel`)**: tells the robot how fast to move (Twist).  
- **Odometry Twist**: robot’s actual current velocity (base frame).  
- **Odometry Pose**: robot’s integrated position & orientation (world frame).  
- **Wheel states**: encoder/joint data → how much each wheel has turned.  
