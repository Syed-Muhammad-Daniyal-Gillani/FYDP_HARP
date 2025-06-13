### harp_setup.sh
#!/bin/bash

set -e

echo "🔧 Adding workspace setup to bashrc..."
grep -qxF 'source ~/FYDP_HARP/install/setup.bash' ~/.bashrc || echo 'source ~/FYDP_HARP/install/setup.bash' >> ~/.bashrc

echo "🧹 Installing dependencies..."
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-argcomplete
sudo apt install -y ros-humble-rosbridge-server python3-pyqt5 python3-pyqt5.qtwebengine
sudo apt install libportaudio2 libportaudiocpp0 portaudio19-dev
sudo apt install qtwayland5
echo "📦 Installing Python dependencies using system Python..."
pip install -r requirements.txt

echo "🔨 Building the ROS 2 workspace..."
colcon build

echo "🛁 Sourcing workspace..."
source install/setup.bash

echo "🚀 HARP setup complete!"
echo "To launch the workspace:"
echo "ros2 launch launch_harp launch_harp.py"