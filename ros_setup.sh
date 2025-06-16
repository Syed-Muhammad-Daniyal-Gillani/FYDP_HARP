### ros_setup.sh
#!/bin/bash
# ROS 2 Humble Development Installation Script for Ubuntu 22.04

set -e

echo "📦 Updating APT and installing prerequisites..."
sudo apt update && sudo apt install -y \


echo "🌐 Setting locale..."
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "Installing prerequisites and packages"
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb

echo "🔁 Updating APT again..."
sudo apt update

echo "📅 Installing ROS 2 base tools (for source builds)..."
sudo apt install -y ros-humble-desktop
sudo apt install ros-dev-tools

echo "🧠 Adding ROS 2 to ~/.bashrc..."
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

echo "✅ ROS 2 Humble development environment installed!"
echo "🔗 Please restart your terminal or run: source ~/.bashrc"