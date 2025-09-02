# 🚀 Visualizing a Differential Drive Robot in RViz (without URDF)

This tutorial explains how to visualize your differential drive robot in **RViz2** using **Gazebo Sim + ROS 2 bridge**, even if you don’t have a URDF model yet.  

---

## 1. Prerequisites
- You have **Gazebo Sim (gz sim)** installed and running a robot with the **diff-drive plugin**.  
- You have **ROS 2** installed.  
- You have the **ros_gz_bridge** package installed.  

---

## 2. Bridge the Topics
The differential drive robot exposes:
- **Velocity command** → `/cmd_vel`
- **Odometry** → `/model/<robot_name>/odometry`

We need to bridge these between Gazebo and ROS 2:

```bash
ros2 run ros_gz_bridge parameter_bridge   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist   /model/<robot_name>/odometry@gz.msgs.Odometry@nav_msgs/msg/Odometry
```

👉 Replace `<robot_name>` with your robot’s name (check using `gz topic -l`).  

---

## 3. Start RViz
Run RViz:
```bash
rviz2
```

---

## 4. Add Displays in RViz

In RViz, click **Add → By topic or type** and select:

1. **Odometry**  
   - Topic: `/model/<robot_name>/odometry`  
   - Shows a moving **arrow** (pose + orientation).  

2. **TF**  
   - Displays coordinate frames (`odom → base_link`).  

3. **Path** (optional)  
   - If you publish a `nav_msgs/Path` from odometry.  
   - Shows the robot’s trajectory line.  

4. **Marker** (optional, advanced)  
   - To visualize velocity vectors as arrows.  

---

## 5. Save Config
Once you add displays:
1. Go to **File → Save Config As…**  
2. Save as `diff_drive_visual.rviz`.  

Now you can reopen with:
```bash
rviz2 -d diff_drive_visual.rviz
```

---

## 6. (Optional) Publish a Path
RViz does not automatically plot paths.  
You can create a simple ROS 2 Python node that subscribes to `/odom` and publishes `nav_msgs/Path` for trajectory visualization.

---

## ✅ Summary
- Use **ros_gz_bridge** to bridge odometry and velocity command.  
- Visualize **Odometry** and **TF** directly in RViz.  
- Add **Path** if you want to track robot trajectory.  
- Save config once and reuse it.  

This works even **without a URDF** — you’ll see arrows and frames instead of a robot model.  
