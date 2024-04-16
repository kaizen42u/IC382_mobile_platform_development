# Making SSH connection to Raspberry PI

`ssh ros-dev@192.168.0.158`

*Password for **ros-dev** is `ros-dev`.

---

Or if you don't want to enter password every time, consider the command `ssh-keygen` and `ssh-copy-id userid@hostname`.

# Command to run `encoder_to_odom.launch`

*Make sure you have already setup the corresponding package and configuration.

```bash
cd development/agv_base_control
source devel/setup.bash
sudo chmod 777 /dev/ttyUSB0
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