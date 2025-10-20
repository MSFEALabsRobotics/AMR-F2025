# RPLIDAR A1 – Quick Start (.md Tutorial)

This guide walks you through installing RoboStudio, drivers, and running a Python visualizer for the **Slamtec RPLIDAR A1** on Windows.


---

## 1) Download & Install RoboStudio

1. Go to **RoboStudio** page: <https://www.slamtec.com/en/ROBOSTUDIO>  
2. Click **Download** and install.
3. Launch RoboStudio, create an account, and **register your email** to sign in.

> RoboStudio is useful to quickly verify that your LIDAR is working before coding.


---

## 2) Install the USB–UART Driver (CP210x)

The A1 uses a USB‑to‑UART bridge (often **CP210x**). Install the **CP210x Windows driver** (available from Silicon Labs / mirrors on GitHub).

- Search for **“CP210x Windows Drivers”** and install.
- After installation, plug the RPLIDAR A1 and check **Device Manager → Ports (COM & LPT)**. Note the **COM#** (e.g., `COM5`).

> If the device shows up with a warning, right‑click → **Update driver** → point to the downloaded CP210x driver.


---

## 3) Connect and Test with RoboStudio

1. Plug in the **RPLIDAR A1** via USB.
2. In RoboStudio, select the detected serial device (your **COM#**) and set **baudrate = 115200** (A1 default).
3. Click **Connect / Start Scan**. You should see live scan data.

> If you don’t see scans: try another USB port, disable power saving on USB hubs, and confirm the COM# in Device Manager.


---

## 4) Python Setup (Windows)

### 4.1 Create an environment (optional but recommended)
```powershell
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
```

### 4.2 Install the RPLIDAR library
```powershell
pip install rplidar-roboticia
# Project page: https://github.com/Roboticia/RPLidar
```

> If you’re on Python 3.13 and hit build issues for unrelated packages, install “Desktop development with C++” in **Visual Studio Installer**.


---

## 5) Python Example – Polar Scatter Animation

The script below opens the RPLIDAR on your **COM port** (update `PORT_NAME`) and shows a live polar scatter of distances with measurement intensity.

```python
#!/usr/bin/env python3
'''Animates distances and measurement quality from RPLIDAR A1'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = 'COM5'        # <- change to your COM port
BAUDRATE = 115200         # A1 default
DMAX = 4000               # mm (4 m typical for A1)
IMIN = 0
IMAX = 50

def update_line(num, iterator, line):
    scan = next(iterator)
    # scan entries: (quality, angle_degree, distance_mm)
    offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
    line.set_offsets(offsets)
    intens = np.array([meas[0] for meas in scan])
    line.set_array(intens)
    return line,

def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)
    try:
        print('Info:', lidar.get_info())
        print('Health:', lidar.get_health())

        fig = plt.figure()
        ax = plt.subplot(111, projection='polar')
        line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                          cmap=plt.cm.Greys_r, lw=0)
        ax.set_rmax(DMAX)
        ax.grid(True)

        iterator = lidar.iter_scans()
        ani = animation.FuncAnimation(fig, update_line,
                                      fargs=(iterator, line), interval=50)
        plt.show()
    finally:
        # Graceful shutdown
        try:
            lidar.stop()
            lidar.stop_motor()
        except Exception:
            pass
        lidar.disconnect()

if __name__ == '__main__':
    run()
```

### Run it
```powershell
python rplidar_polar_live.py
```

