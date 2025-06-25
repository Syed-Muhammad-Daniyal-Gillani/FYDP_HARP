# ROS2 Environment Setup for HARP 🤖

---

## 🤖 Auto Installation Guide

---

Clone the repo if you haven't already:

```bash
git clone https://github.com/smdaniyalgillani/harp-humanoid-robot.git
cd harp-humanoid-robot/
```

### ⚙️ ROS2 Setup

> **📝 Requires:** Ubuntu 22.04

Run the automated ROS2 installer in terminal:

```bash
bash ros_setup.sh
```
or
```bash
./ros_setup.sh
```
---

### 🤖 HARP Setup

> **📝 Requires:** ROS2 Humble already installed and sourced

Run the provided setup script in terminal:

```bash
bash harp_setup.sh
```
or
```bash
./harp_setup.sh
```
---

## 🚀 Running the Project

Once done, restart terminal and run:

```bash
ros2 launch launch_harp launch_harp.py
```

---

## 📦 Manual Installation Guide

---

### 🔧 ROS2 Setup

> **📝 Prerequisite:** Ensure you are running **Ubuntu 22.04**.

Install **ROS2 Humble** by following the official instructions:  
🔗 [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

### ⚙️ HARP Setup

> **📝 Prerequisite:** ROS2 Humble must be installed before proceeding.

Clone the repository:

```bash
git clone https://github.com/Syed-Muhammad-Daniyal-Gillani/harp-humanoid-robot.git
cd harp-humanoid-robot/
```

Then run the following shell commands step-by-step:

```bash
sudo apt update
```
```bash
sudo apt install -y python3-colcon-common-extensions
```
```bash
sudo apt install -y python3-pip
```

```bash
sudo apt install -y python3-rosdep
```

```bash
sudo apt install -y python3-argcomplete
```

```bash
sudo apt install -y ros-humble-rosbridge-server
```

```bash
sudo apt install -y python3-pyqt5 python3-pyqt5.qtwebengine
```

```bash
sudo apt install -y libportaudio2 libportaudiocpp0 portaudio19-dev
```

```bash
sudo apt install -y qtwayland5
```

```bash
pip install -r requirements.txt
```

### 🔧 Add workspace setup to `.bashrc`

```bash
grep -qxF 'source ~/harp-humanoid-robot/install/setup.bash' ~/.bashrc || echo 'source ~/harp-humanoid-robot/install/setup.bash' >> ~/.bashrc
```

### 🔨 Build the ROS 2 workspace

```bash
colcon build
```

---

### 🛁 Source the workspace

```bash
source install/setup.bash
```

---

### 🚀 Launch the HARP system

```bash
ros2 launch launch_harp launch_harp.py
```
