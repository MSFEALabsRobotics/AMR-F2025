# 🛩️ Setting up Raspberry Pi for AirSim

This guide explains how to prepare a **Raspberry Pi (Lite or Desktop)** to connect with **AirSim** and process image or MAVLink data.

---

## 1️⃣ Download Required Tools

### 🧰 Raspberry Pi Imager
Download and install the official imager from:  
🔗 [https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/)

### 💿 Raspberry Pi OS Image
Choose either:
- **Raspberry Pi OS Lite** (for headless setup)
- **Raspberry Pi OS with Desktop** (if you prefer GUI setup)  
🔗 [https://www.raspberrypi.com/software/operating-systems/](https://www.raspberrypi.com/software/operating-systems/)

---

## 2️⃣ Flash the SD Card

1. Open **Raspberry Pi Imager**  
2. Select the OS and the SD card  
4. Click **Write** to flash the image  
5. Insert the SD card into the Pi

---

## 3️⃣ First Boot

1. Connect:
   - HDMI display  
   - Keyboard
   - Power supply  
2. Boot up and open terminal  
3. Run configuration:
   ```bash
   sudo raspi-config
   ```
   - Set Wi-Fi country as US and connect to network  
   - Enable **SSH** (used for remote connection)
   - ssh username@ipaddress

---

## 4️⃣ Basic Setup Commands

Update and install Python essentials:
```bash
sudo apt update
sudo apt install python3-pip python3-numpy python3-msgpack python3-setuptools python3-wheel -y
```



