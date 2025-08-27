# WSL & Gazebo Setup Guide

## 🖥️ Install WSL
```bash
wsl --install --no-distribution
```

---

## 📂 Create a New Folder
```bash
mkdir "C:\WSL\MyNewImage"
```

---

## 📦 Import the Tar Image
```bash
wsl --import MyNewImage "FolderPath" "ImagePath"
```

**Example:**
```bash
wsl --import MyNewImage "C:\WSL\MyNewImage" "C:\Users\samer\Documents\AUB\AMR\mywsl.tar"
```

---

## 🤖 Test Gazebo
```bash
gz sim
```

---

## 📐 Open Shapes.sdf
```bash
gz sim shapes.sdf
```

---

## 🐙 Using GIt
```bash
git clone <repository_url>
```

**Example:**
```bash
git clone https://github.com/MSFEALabsRobotics/AMR-F2025
```

---
