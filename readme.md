# How to use

Download this repo and unzip it.

Rename folder `IC382_mobile_platform_development` to `development`.

Upload the `development` folder to your ROS capable Raspberry PI's current user folder.

There are two files that might be of interest:
 - agv_base_control/src/controller/src/controller.py
 - agv_base_control/src/robot_encoder_odom/src/encoder_to_odom.cpp

# Making SSH connection to Raspberry PI (Linux only)

* Windows users please use [PuTTY](https://www.putty.org/) or [Tera Term](https://teratermproject.github.io/index-en.html) or anything that accepts SSH connection.

`ssh ros-dev@192.168.0.158`

*Password for **ros-dev** is `ros-dev`.

---

Or if you don't want to enter password every time, consider the command `ssh-keygen` and `ssh-copy-id userid@hostname`.

# Command to run `encoder_to_odom.launch`

*Make sure you have already setup the corresponding package and configuration.

On the PI terminal, run the following command:
```bash
cd development/agv_base_control
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB0
catkin_make
roslaunch robot_encoder_odom agv_base_control_odom.launch
```

*To exit the program, spam click `CTRL+C` until you are back to CLI.

# Command to run `controller.py`

*Make sure you have already setup the corresponding package and configuration.

*Make sure **encoder_to_odom.launch** is running before running **controller.py**

Open another terminal, login, then run the following command:
```bash
cd development/agv_base_control
source devel/setup.bash
rosrun controller controller.py
```

*To exit the program, press `CTRL+Z`.