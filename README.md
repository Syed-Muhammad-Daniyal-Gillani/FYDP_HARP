# ROS2 Workspace with Virtual Environment (NOT RECOMMENDED see notes)

This guide sets up a Python virtual environment (venv) for your ROS2 workspace while preventing conflicts during `colcon build`. It also ensures that Python dependencies inside the virtual environment are properly recognized.

Replace FYDP HARP with the folder you're environment is in.
## Setup Instructions
### 1️⃣ Create a Virtual Environment
```bash
python3 -m venv venv
```
This creates a `venv` inside `FYDP_HARP`.

### 2️⃣ Create a ROS2 Package
If you want to create a new ros2 package:
```bash
cd src
ros2 pkg create my_package --build-type ament_python --dependencies rclpy
```

### 3️⃣ Prevent Colcon from Scanning venv
```bash
touch ~/FYDP_HARP/venv/COLCON_IGNORE
```
This prevents `colcon` from processing the virtual environment.

### 4️⃣ Activate venv & Install Dependencies
```bash
source ~/FYDP_HARP/venv/bin/activate
pip install -r requirements.txt  # Install dependencies
pip install rclpy  # Ensure ROS2 dependencies are installed
```

### 5️⃣ Set PYTHONPATH
```bash
export PYTHONPATH="$HOME/FYDP_HARP/venv/lib/python3.10/site-packages:$PYTHONPATH"
```
To make this persistent, add it to `~/.bashrc`:
```bash
echo 'export PYTHONPATH="$HOME/FYDP_HARP/venv/lib/python3.10/site-packages:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc
```

### 6️⃣ Build the Workspace
```bash
cd ~/FYDP_HARP
colcon build
```

### 7️⃣ Run Your ROS2 Node
```bash
source install/setup.bash
ros2 run my_package my_node
```

## Notes
- Due to compatibility issues between `ROS2` and `venv`, You may need to run `rosbridge server` independently outisde the venv. In order to do so, comment out the following lines from src/launch_harp/launch/launch_harp.py and follow the `How to Launch` instructions.
```bash
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='ROSBridge_Local_Server',
            output = 'screen'            
        )
```
- If you encounter `pytest` warnings or errors, comment out related lines in `setup.py`.
- This method ensures colcon ignores the virtual environment while still recognizing dependencies.
- Works with other ROS2 distros as long as Python versions match.

