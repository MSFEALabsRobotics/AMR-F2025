# 🚀 Connecting LEGO NXT to WSL (AMR Image)

These instructions explain how to attach your LEGO Mindstorms NXT brick (USB) to a specific WSL distribution (e.g., `AMR`).

---

## 1. Install `usbipd-win` on Windows
Open **PowerShell (Admin)** and run:

```powershell
winget install usbipd
```

Check installation:

```powershell
usbipd --version
```

---

## 2. Plug in the LEGO NXT brick
After plugging in the device, list USB devices:

```powershell
usbipd list
```

Example output:

```
BUSID  VID:PID    DEVICE                              STATE
1-7    0694:0002  LEGO Group Mindstorms NXT           Not shared
1-5    1bcf:2ced  Hy-FHD(9807)-Camera                 Not shared
```

Note the **BUSID** of your LEGO NXT (e.g., `1-7`).

---

## 3. Bind and attach NXT to WSL (AMR)
Still in PowerShell (Admin):

```powershell
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID> --distribution AMR
```

Example (Make sure you have at least one WSL terminal Open):
```powershell
usbipd bind --busid 1-7
usbipd attach --wsl --busid 1-7   
```

---

## 4. Verify inside WSL
Open your AMR distribution:

```powershell
wsl -d AMR
```

Check USB devices:

```bash
lsusb
```

Expected output:
```
Bus 001 Device 004: ID 0694:0002 Lego Group Mindstorms NXT
```

---

## 5. Install Python support in WSL
Run inside `AMR`:

```bash
sudo apt update
sudo apt install libusb-0.1-4 libusb-dev python3-pip -y
pip install nxt-python
```

---

## 6. Test the connection
Run this quick Python test:

```bash
python3 - <<'EOF'
import nxt.locator
brick = nxt.locator.find()
print("Connected to:", brick.get_device_info())
EOF
```

If successful, it will print the NXT brick name and details.

---

✅ Now you can run your **Hello Tacho** or any control scripts directly from WSL.
