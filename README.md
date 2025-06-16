# ROS2 Environment Setup for HARP 🤖

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
git clone https://github.com/CEME-HARP/FYDP_HARP.git
cd FYDP_HARP
```

Then run the following shell commands step-by-step:

```bash
set -e

echo "🔧 Adding workspace setup to bashrc..."
grep -qxF 'source ~/fydp_harp/install/setup.bash' ~/.bashrc || echo 'source ~/fydp_harp/install/setup.bash' >> ~/.bashrc

echo "🧹 Installing dependencies..."
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-argcomplete
sudo apt install -y ros-humble-rosbridge-server python3-pyqt5 python3-pyqt5.qtwebengine
sudo apt install -y libportaudio2 libportaudiocpp0 portaudio19-dev
sudo apt install -y qtwayland5

echo "📦 Installing Python dependencies using system Python..."
pip install -r requirements.txt

echo "🔨 Building the ROS 2 workspace..."
colcon build

echo "🛁 Sourcing workspace..."
source install/setup.bash

echo "🚀 HARP setup complete!"
echo "To launch the workspace:"
echo "ros2 launch launch_harp launch_harp.py"
```

To run the project:

```bash
ros2 launch launch_harp launch_harp.py
```

If needed, launch rosbridge manually in a separate terminal:

```bash
ros2 run rosbridge_server rosbridge_websocket
```

> 🔧 *Make sure to comment out the rosbridge node in `launch_harp.py` if running manually.*

---

## 🤖 Auto Installation Guide

---

### ⚙️ ROS2 Setup

> **📝 Requires:** Ubuntu 22.04

Run the automated ROS2 installer:

```bash
bash ros_setup.sh
```

---

### 🤖 HARP Setup

> **📝 Requires:** ROS2 Humble already installed and sourced

Clone the repo if you haven't already:

```bash
git clone https://github.com/CEME-HARP/FYDP_HARP.git
cd FYDP_HARP
```

Then run the provided setup script:

```bash
bash harp_setup.sh
```

---

## 🚀 Running the Project

Once everything is set up, simply run:

```bash
ros2 launch launch_harp launch_harp.py
```

And you're good to go!