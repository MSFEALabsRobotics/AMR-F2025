# Add a 2D LiDAR in Gazebo (gz-sim), Bridge to ROS 2, and Visualize in RViz


---

## 1) Add a LiDAR reference frame to the robot

Place this **under the `<model name="vehicle_blue">` tag** in your world or model SDF:

```xml
<frame name="lidar_frame" attached_to='chassis'>
  <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```

- This creates a rigid frame named `lidar_frame` attached to `chassis`, located **0.8 m forward**, **0 m lateral**, **0.5 m up**, with **no rotation**.

---

## 2) Enable the Sensors system in the world

Put this **under `<world>`** (sibling of models) so Gazebo publishes sensor data:

```xml
<plugin
    filename="gz-sim-sensors-system"
    name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

---

## 3) Add the 2D GPU LiDAR under the chassis link

Add this **inside the `<link name="chassis">`** (or the link that should carry the sensor):

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
  <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
  <topic>lidar</topic>
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.396263</min_angle>
        <max_angle>1.396263</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>0.01</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <always_on>1</always_on>
  <visualize>true</visualize>
</sensor>
```

### What the parameters mean (and how to tune)

- **`<pose relative_to='lidar_frame'>…</pose>`**: Mount pose relative to the frame you added earlier.
- **`<topic>`**: Name of the Gazebo transport topic for the LiDAR output (`/lidar`).
- **`<update_rate>`**: Frequency in **Hz**. Here `10` → new scan every **0.1 s**.
- **`<horizontal>` / `<vertical>`**: Number of simulated rays and angular limits.
  - **`<samples>`**: number of rays per sweep (horizontal) / per column (vertical).
  - **`<resolution>`**: multiplied by `samples` to get the number of returned points.
  - **`<min_angle>` / `<max_angle>`**: angular span in radians (here ≈ ±80°).
- **`<range>`**: per-ray distance properties.
  - **`<min>` / `<max>`**: minimum/maximum return distance (m).
  - **`<resolution>`**: linear distance resolution.
- **`<always_on>`**: when `true`, runs at the given `update_rate` continuously.
- **`<visualize>`**: draws the sensor frustum/points in the GUI (**debug-only** cost).

**Common tuning tips**:
- Wider FoV → increase `min_angle`/`max_angle` span (e.g., ±135° is `±2.35619` rad).
- Higher angular density → increase `samples` (tradeoff with performance).
- Faster scans → increase `update_rate` (CPU/GPU cost goes up).
- Range fidelity → adjust `<range><resolution>` and `<max>` to your scene scale.

---

## 4) Run Gazebo and confirm topics

Start your world (SDF) as usual, then in a terminal:

```bash
gz topic -l
```

You should see entries like:
```
/lidar
/lidar/points
```
(`points` exists when the GPU LiDAR also publishes a point cloud).

---

## 5) Bridge LiDAR to ROS 2

Use the **parameter bridge** to convert Gazebo messages into ROS 2 messages:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

> Tip: Add `--ros-args -r __ns:=/my_robot` if you want a namespace.

Verify in ROS 2:

```bash
# list topics
ros2 topic list

# inspect some data
ros2 topic echo /lidar --once
ros2 topic echo /lidar/points --once
```

---

## 6) Visualize in RViz

Open **RViz** and add these displays:

**LaserScan**
- *Add → By display type →* **LaserScan**
- **Topic**: `/lidar`
- **Style**: *Points*
- **Size (Pixels)**: `2–4`
- **Color Transformer**: *Intensity* (if available) or *Flat Color*

**PointCloud2**
- *Add → By display type →* **PointCloud2**
- **Topic**: `/lidar/points`
- **Style**: *Points*
- **Size (m)**: `0.02–0.05`
- **Color Transformer**: *Intensity* (or *Z* / *RGB* if present)

> ⚠️ Ensure TF is available (`map`/`odom`/`base_link` etc.) so RViz can place data correctly. You can also set **Fixed Frame** to the robot’s base frame used by your TF tree.

---

## 7) (Optional) Surround the robot with a simple maze

If you already have a maze model, include it in the world as a sibling to `vehicle_blue`:

```xml
<include>
  <uri>model://my_maze</uri>
  <name>maze</name>
  <pose>0 0 0 0 0 0</pose>
</include>
```

> You can build a quick maze from boxes using `<model><link><collision><geometry><box/></…>` repeated in your SDF, or drop a ready model into `~/.gz/models/my_maze` and reference it by `model://my_maze`.

---

## 8) Simple ROS 2 node: scatter-plot LiDAR points + front-distance estimate

This node subscribes to **`/lidar/points`** (`sensor_msgs/PointCloud2`), converts to XYZ, renders a **live scatter plot** (top-down), and **estimates** the **minimum forward distance** (within ±15°), plotting it live as a time series in a second window.

> Requires Python packages: `rclpy`, `numpy`, `matplotlib`, `sensor_msgs_py` (or `ros2`’s `ros_numpy` alternative).

### Create a package (quick start)

```bash
# From your ROS 2 workspace (e.g., ~/ros2_ws/src)
ros2 pkg create --build-type ament_python lidar_viz

# Put the script below into: lidar_viz/lidar_viz/lidar_scatter.py
# And update setup.cfg / setup.py to install entry point lidar_scatter = lidar_viz.lidar_scatter:main
```

**`lidar_viz/lidar_viz/lidar_scatter.py`**

```python
#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import matplotlib.pyplot as plt
from collections import deque
import threading

FWD_DEG_WIDTH = 15.0    # +/- degrees for "front" sector
HISTORY_SEC   = 30.0    # seconds to keep in distance history
RATE_HZ       = 10      # plot refresh rate

class LidarScatter(Node):
    def __init__(self):
        super().__init__('lidar_scatter')
        self.sub = self.create_subscription(PointCloud2, '/lidar/points', self.cb, 10)
        self.lock = threading.Lock()
        self.xy = np.empty((0, 2))
        self.dist_hist = deque(maxlen=int(HISTORY_SEC * RATE_HZ))
        self.time_hist = deque(maxlen=int(HISTORY_SEC * RATE_HZ))

        # Matplotlib interactive windows
        plt.ion()
        self.fig1 = plt.figure(num='LiDAR XY Scatter (top view)')
        self.ax1 = self.fig1.add_subplot(111)
        self.sc = self.ax1.scatter([], [])
        self.ax1.set_aspect('equal', adjustable='box')
        self.ax1.set_xlabel('X (m)')
        self.ax1.set_ylabel('Y (m)')
        self.ax1.grid(True)

        self.fig2 = plt.figure(num='Front Distance Estimate')
        self.ax2 = self.fig2.add_subplot(111)
        self.line, = self.ax2.plot([], [], linewidth=2)
        self.ax2.set_xlabel('Samples (≈time)')
        self.ax2.set_ylabel('Min distance in ±15° (m)')
        self.ax2.grid(True)

        self.timer = self.create_timer(1.0 / RATE_HZ, self.redraw)

    def cb(self, msg: PointCloud2):
        # Convert PointCloud2 to Nx2 XY (in sensor frame)
        pts = np.array([p for p in point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)])
        if pts.size == 0:
            return
        xy = pts[:, :2]  # ignore z for 2D plot

        # Estimate "front" distance within ±FWD_DEG_WIDTH
        # Convert to polar
        angles = np.arctan2(xy[:,1], xy[:,0])  # rad
        mask = np.abs(np.degrees(angles)) <= FWD_DEG_WIDTH
        front_pts = xy[mask]
        front_min = np.inf
        if front_pts.size > 0:
            front_min = np.min(np.linalg.norm(front_pts, axis=1))

        with self.lock:
            self.xy = xy
            self.dist_hist.append(front_min if np.isfinite(front_min) else np.nan)
            self.time_hist.append(self.get_clock().now().nanoseconds * 1e-9)

    def redraw(self):
        with self.lock:
            xy = self.xy.copy()
            dh = list(self.dist_hist)

        if xy.size > 0:
            self.ax1.cla()
            self.ax1.set_aspect('equal', adjustable='box')
            self.ax1.set_xlabel('X (m)')
            self.ax1.set_ylabel('Y (m)')
            self.ax1.grid(True)
            self.ax1.scatter(xy[:,0], xy[:,1], s=3)
            self.ax1.set_title('LiDAR XY Scatter (sensor frame)')

        if len(dh) > 0:
            self.ax2.cla()
            self.ax2.grid(True)
            self.ax2.set_xlabel('Samples (≈time)')
            self.ax2.set_ylabel('Min distance in ±15° (m)')
            self.ax2.plot(dh, linewidth=2)
            self.ax2.set_title('Front Distance Estimate')

        plt.pause(0.001)

def main():
    rclpy.init()
    node = LidarScatter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**`setup.py` (minimal)**

```python
from setuptools import setup

package_name = 'lidar_viz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='LiDAR scatter plot and front-distance estimate',
    entry_points={
        'console_scripts': [
            'lidar_scatter = lidar_viz.lidar_scatter:main',
        ],
    },
)
```

**`package.xml` (minimal)**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>lidar_viz</name>
  <version>0.0.1</version>
  <description>LiDAR scatter plot and front-distance estimate</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-matplotlib</exec_depend>
</package>
```

Build and run:

```bash
# From your workspace root
colcon build --symlink-install
source install/setup.bash

# Run the node (after starting the bridge)
ros2 run lidar_viz lidar_scatter
```

> Notes:
> - The scatter is in the **sensor frame** (X forward, Y left). If you prefer **map/odom/base_link** frames, transform points with TF before plotting.
> - If you want **LaserScan** instead, subscribe to `/lidar` and convert `(range, angle)` → `(x, y)` via `x = r*cos(θ)`, `y = r*sin(θ)`.

---

## 9) Quick checklist

- [ ] Added `<frame name="lidar_frame" …>` under `vehicle_blue`.
- [ ] Enabled `gz::sim::systems::Sensors` plugin in the `<world>`.
- [ ] Created `<sensor type="gpu_lidar">` under the **chassis** link.
- [ ] `gz topic -l` shows `/lidar` and `/lidar/points`.
- [ ] Bridged to ROS 2 with `ros_gz_bridge`.
- [ ] RViz shows **LaserScan** and/or **PointCloud2**.
- [ ] (Optional) Maze model included and visible.
- [ ] Python node running and plotting live data.

Happy scanning! 🎯
