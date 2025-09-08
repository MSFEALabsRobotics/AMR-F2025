# 🚗 RVIZ Tutorial

This tutorial shows how to run a differential drive robot in **Gazebo Sim**, bridge the topics into **ROS 2**, and visualize everything in **RViz 2**.  

---

## 1. Start your robot in Gazebo
Make sure you have a robot `.sdf` file (example: `vehicle_blue.sdf`) and run:

```bash
gz sim vehicle_blue.sdf
```

---

## 2. Start ROS ↔ Gazebo Bridges
Bridge **odometry** and **velocity commands**:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

---

## 3. Add Missing TF with Manual Bridge
In this version of Gazebo → ROS 2, the TF transform is not automatically provided.  
We will create a manual bridge from **Odometry → TF**.

### Save this script as `BridgeOdomToTF.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        # Use odometry header info
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id   # vehicle_blue/odom
        t.child_frame_id = msg.child_frame_id     # vehicle_blue/chassis

        # Pose → transform
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Run it:
```bash
python3 BridgeOdomToTF.py
```

---

## 4. Check Available ROS 2 Topics
List all topics:

```bash
ros2 topic list
```

Check that `/tf` is being published:

```bash
ros2 topic echo /tf
```

---

## 5. Start RViz2
Launch RViz:

```bash
rviz2 rviz2
```

### Configure RViz:
1. **Fixed Frame** → set to `odom` (from odometry frame).  
2. Add display: **Odometry** → set topic to `/model/vehicle_blue/odometry`.  
3. Add display: **TF** → visualize the transform tree (`odom` → `chassis`).  

You should now see your robot’s odometry and TF frames updating live.

---

✅ Done! You now have Gazebo ↔ ROS 2 odometry + velocity bridges, a manual TF broadcaster, and visualization in RViz2.
