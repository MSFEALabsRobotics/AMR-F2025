# Pose With Covariance in ROS 2 + RViz (Student Mini‑Guide)

## 1) What is a “pose with covariance”?
- **Pose** = position *(x, y, z)* + orientation *(roll, pitch, yaw)* of the robot in some frame (e.g., `map`, `odom`).  
- **Covariance** = a 6×6 matrix that expresses **uncertainty** (variance and cross‑correlations) on those six variables.  
- In ROS 2, the standard message is **`geometry_msgs/PoseWithCovarianceStamped`**. It carries:
  - `header` (time stamp + frame id)
  - `pose.pose` (the pose)
  - `pose.covariance` (a **flattened** 36‑length array = the 6×6 covariance in **row‑major** order over `[x, y, z, roll, pitch, yaw]`).

> Intuition: Small variances ⇒ the pose estimate is confident (tight ellipses in RViz). Large variances ⇒ the estimate is uncertain (wide ellipses).

---

## 2) The custom “bridge” / relay node
This node **subscribes** to an odometry topic and **republishes** it as `PoseWithCovarianceStamped`, injecting a simple 2‑D covariance so RViz can draw the uncertainty ellipse.

### Topics
- **In**: `/model/vehicle_blue/odometry` (`nav_msgs/Odometry`)
- **Out**: `/belief/pose` (`geometry_msgs/PoseWithCovarianceStamped`)

### Key lines explained
```python
cov = [0.0]*36        # Make a zero 6×6 matrix (flattened to length 36)
cov[0]  = 0.04        # var(x)   -> σ = sqrt(0.04)  = 0.20 m
cov[7]  = 0.8         # var(y)   -> σ = sqrt(0.8)   ≈ 0.894 m
cov[14] = 1e-9        # var(z)   -> ~0 to force a “2‑D” interpretation
cov[21] = 1e-9        # var(roll)
cov[28] = 1e-9        # var(pitch)
cov[35] = 0.4         # var(yaw) -> σ = sqrt(0.4)   ≈ 0.632 rad ≈ 36.2°
msg.pose.covariance = cov
```
- **Why indices 0,7,14,21,28,35?** Those are the **diagonal** cells of the 6×6 matrix when flattened row‑major:
  - index \(i,i\) sits at `i*6 + i`. For `i=0..5` ⇒ `0,7,14,21,28,35`.
- **Numbers meaning** (you can tune them!):
  - `var(x)=0.04` ⇒ **±0.20 m** 1‑σ along **x**.
  - `var(y)=0.8`  ⇒ **±0.894 m** 1‑σ along **y** (more uncertainty side‑to‑side).
  - `var(yaw)=0.4` ⇒ **±0.632 rad ≈ 36°** 1‑σ on heading.
  - Very small `z/roll/pitch` variances keep the display effectively **2‑D**.

> **Note on comments**: Make sure variance ↔ standard deviation comments match the actual numbers (`σ = sqrt(variance)`). The code above is **numerically** correct; if your inline comments say otherwise, fix the comments, not the math.

---

## 3) Run the relay and check the topic (terminal)
1) Run the node (pick **one** of the options):
```bash
# Option A: run as a Python script (same shell where your ROS 2 env is sourced)
python3 add_cov_relay.py

# Option B: if you installed it as a ROS 2 package executable:
ros2 run my_pkg add_cov_relay
```
2) Verify the output message type:
```bash
ros2 topic type /belief/pose
# -> geometry_msgs/msg/PoseWithCovarianceStamped
```
3) Peek one message:
```bash
ros2 topic echo /belief/pose --once
```
4) Show the interface definition (handy to study fields):
```bash
ros2 interface show geometry_msgs/msg/PoseWithCovarianceStamped
```

> **Case matters**: the topic is `/belief/pose` (all‑lowercase). If you type `/bELIEF/poSE`, it won’t match.

---

## 4) Subscribe quickly from the terminal
- To continuously watch:
```bash
ros2 topic echo /belief/pose
```
- To throttle (e.g., once per second):
```bash
ros2 topic echo /belief/pose --qos-durability volatile --qos-reliability reliable | stdbuf -oL awk 'NR % 10 == 0'
```
*(or just use `--once` as shown above to print a single sample).*

---

## 5) Visualize in **RViz2**
1) Launch RViz2, set **Fixed Frame** (usually `map` or `odom`) to match `msg.header.frame_id`.
2) Add a display:
   - **Option A:** *PoseWithCovariance*  
     Click **Add** → **By display type** → **Pose** → **PoseWithCovariance**.
   - **Option B:** *Odometry* (can also draw pose + covariance if you point it to an `Odometry` topic) — not needed here since we already publish `PoseWithCovariance`.
3) In the display’s **Topic**, select:  
   **`/belief/pose`**
4) Expand **Covariance** settings and tweak:
   - **Position** → *Enable* (shows the XY ellipse)
   - **Orientation** → *Enable* (shows the yaw cone/ellipse)
   - **Scale** / **Alpha**: make the ellipse visible but not overwhelming.
5) If nothing appears:
   - Check that **Fixed Frame** matches the message frame.
   - Confirm messages are arriving with `ros2 topic hz /belief/pose`.
   - Ensure the covariance isn’t all zeros (RViz hides zero‑area ellipses).

---

## 6) Quick FAQ
- **Why add covariance at all?** RViz uses it to draw **uncertainty ellipses**. Many filters (EKF/UKF/PF) also expect proper covariances.
- **Is the matrix only diagonal?** Here we set only the diagonal (**no cross‑correlation**). You can fill **off‑diagonals** (e.g., `cov[1]` for `cov(x,y)`) if your estimator knows them.
- **2‑D robot?** Set tiny variances for `z/roll/pitch` and focus on `x, y, yaw`.

Happy plotting! 🎯
