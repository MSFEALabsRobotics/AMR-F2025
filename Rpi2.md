# ROS 2 in WSL ↔ ROS 2 on Raspberry Pi (Same Network)

Yes, **ROS 2 in WSL can talk to ROS 2 on a Raspberry Pi** on the same network – but only if you set things up carefully.

Here’s the practical version:

---

## 1. Check what you’re using: WSL1 vs WSL2

### WSL1

- Shares the **same IP as Windows**.  
- Networking behaves more like a “normal” app on Windows.  
- Often easier for ROS 2 discovery on the LAN.

### WSL2

- Has its **own virtual IP** behind NAT (like a little VM).  
- ROS 2 discovery (DDS multicast) may fail across machines unless you tweak things.

You can check in PowerShell:

```powershell
wsl -l -v
```

---

## 2. Common settings you must align

On **both** WSL and Raspberry Pi:

```bash
export ROS_DOMAIN_ID=10        # same on both
export ROS_LOCALHOST_ONLY=0    # must be 0 for multi-machine
```

You can put these in `~/.bashrc` on both sides.

Also make sure:

- Same **ROS 2 distro** (e.g., Humble on both).
- Same **RMW** if possible, e.g. FastDDS or CycloneDDS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # or cyclone
```

---

## 3. If you’re on WSL2 (most likely)

This is where things get tricky.

### Option A – It “just works” (sometimes)

If your Windows + WSL2 + Pi are on the same LAN and multicast passes through, sometimes ROS 2 discovery works out of the box once you set:

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_DOMAIN_ID=10
```

Then test:

On the **Pi**:

```bash
ros2 run demo_nodes_cpp talker
```

In **WSL**:

```bash
ros2 topic list
ros2 run demo_nodes_cpp listener
```

If you see topics/messages → great, you’re done.

---

### Option B – Use a Discovery Server (more robust)

If discovery doesn’t work, configure a **Fast DDS Discovery Server** (or CycloneDDS equivalent). Typical pattern:

1. Pick one machine (Pi or WSL) as the **discovery server** (fixed IP).  
2. Start the discovery server there.  
3. Configure both sides as **clients** pointing to that server.

This bypasses multicast and works nicely across NAT / weird networks.

---

## 4. Firewalls & pings

- Make sure **Windows firewall** is not blocking ROS 2 traffic.
- From Pi → ping WSL IP and vice-versa:

```bash
# in WSL
ip addr   # find eth0 or similar ip, e.g. 172.27.x.x

# on Pi
ping <WSL_IP>

# in WSL
ping <PI_IP>
```

If they can’t ping each other, ROS 2 definitely won’t work.

---

## 5. Quick summary

- ✅ Yes, you **can** connect ROS 2 in WSL with ROS 2 on a Raspberry Pi.
- Make sure: same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, compatible RMW, firewall OK.
- On **WSL2**, if automatic discovery fails, use a **DDS discovery server** (or switch to WSL1 / native Windows ROS 2).

If you tell me:
- which ROS 2 distro (Humble/Jazzy?),
- WSL version (1 or 2),
- and if you prefer FastDDS or Cyclone,

I can give you a minimal “copy-paste” config for your exact case.
