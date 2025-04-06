# ROS2 Environment Setup
## Prerequisites
- Ensure ROS2 Humble has been properly installed. Follow the setup guide `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html`
## Initial Setup
### Run the following commands in terminal
- Clone the repo using 
```bash
git clone git@gitlab.com:fydp_harp/fydp_harp.git
```
- Run 
``` bash
 gedit ~/.bashrc
```
 and add the following line at the end of the document. Then restart the terminal.
``` bash 
source ~/ros2_harp/install/setup.bash
```

- Ensure rosbridge is installed.
``` bash
 sudo apt install ros-humble-rosbridge-server
```

- Go to the repo directory. 
``` bash
 cd ros2_harp
 ``` 
 
- If you want to create a virtual environment, refer to its guide below, otherwise run 
``` bash
 pip install -r requirements.txt #to install all dependencies
```  
- Build the project using
``` bash
 colcon build
```
 Run 
``` bash
 source install/setup.bash
```

## Launch workspace (no venv)
- Run 
``` bash
 ros2 launch launch_harp launch_harp.py
```
 to run the project.

## Launch workspace (with venv)
- Open a secondary terminal and type ```deactivate``` if venv activates automatically to deactivate it.
- Run 
``` bash
 ros2 run rosbridge_server rosbridge_websocket
```
. (Make sure to comment out rosbridge node from launch file given in guide notes below)
- In another terminal with venv activated, run ``` bash ros2 launch launch_harp launch_harp.py``` to run the project.


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
- If you encounter `pytest` warnings or errors, comment out related lines in `setup.py` of each individual module folder.
