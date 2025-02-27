# ROS2 Environment Setup
## Initial Setup
- Pull the repo using `git pull https://github.com/CEME-HARP/ros2-desktop-integration.git`
- Run `gedit ~/.bashrc` and add the following line at the end of the document `source ~/ROS_FYDP/install/setup.bash`
- Go to the repo directory by typing `cd ros2humble_integration` in terminal
- If you want to create a virtual environment, refer to its guide below, otherwise run `pip install -r requirements.txt`  #to install all dependencies
- Run `colcon build` and then source `install/setup.bash` in terminal.

## Launch workspace
- Run `ros2 launch launch_harp launch_harp.py` to run the project.



# ROS2 Workspace with Virtual Environment Setup (NOT RECOMMENDED see notes)

This guide sets up a Python virtual environment (venv) for your ROS2 workspace while preventing conflicts during `colcon build`. It also ensures that Python dependencies inside the virtual environment are properly recognized.

Replace FYDP HARP with the folder you're environment is in.
## Setup Instructions
### 1️⃣ Create a Virtual Environment
```bash
python3 -m venv venv
```
This creates a `venv` inside `FYDP_HARP`.

### 2️⃣  Prevent Colcon from Scanning venv
```bash
touch ~/FYDP_HARP/venv/COLCON_IGNORE
```
This prevents `colcon` from processing the virtual environment.

### 3️⃣ Activate venv & Install Dependencies
```bash
source ~/FYDP_HARP/venv/bin/activate
pip install -r requirements.txt  # Install dependencies
pip install rclpy  # Ensure ROS2 dependencies are installed
```

### 4️⃣ Set PYTHONPATH
```bash
export PYTHONPATH="$HOME/FYDP_HARP/venv/lib/python3.10/site-packages:$PYTHONPATH"
```
To make this persistent, add it to `~/.bashrc`:
```bash
echo 'export PYTHONPATH="$HOME/FYDP_HARP/venv/lib/python3.10/site-packages:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc
```

### 5️⃣ Build the Workspace
```bash
cd ~/FYDP_HARP
colcon build
```

### 6️⃣ Run Your ROS2 Node
```bash
source install/setup.bash
ros2 run my_package my_node
```

## Notes
- Due to compatibility issues between `ROS2` and `venv`, You may need to run `rosbridge server` independently outside the venv. In order to do so, comment out the following lines from src/launch_harp/launch/launch_harp.py and follow the `How to Launch` instructions.
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

