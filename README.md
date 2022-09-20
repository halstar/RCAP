
A homemade mecanum wheels based robot car, running Robot OS on a Raspberry Pi, with an additional STM32 board, dedicated to motors & making possible to drive the car with a PS2 controller.

This repository gathers the Raspberry Pi files, a.k.a. the ROS car application (RCAP).

## Install Ubuntu Mate 20.04.3 LTS (a.k.a. Focal Fossa)

## Update Ubuntu & add some regular tools

- sudo apt-get update
- sudo apt-get upgrade
- sudo apt-get install vim pyyaml
- sudo apt-get install libssl-dev
- sudo apt-get install i2c-tools libi2c-dev smbus
- sudo apt-get install curl gnupg2 lsb-release
- sudo apt-get install libpython3-dev python3-pip
- sudo pip3    install transforms3d

## Setup a local network between the robot car and local Windows PC

- sudo ufw disable
- sudo ifconfig enp0s8 192.168.1.45
- sudo route add default gw 192.168.1.254 enp0s8
- sudo /etc/init.d/networking restart

## Setup SSH

- sudo apt-get openssh-server

## Setup VNC

- sudo apt-get install tightvncserver


- tightvncserver
> Enter and confirm  password
> Enter no view-only password


- sudo vim /etc/systemd/system/tightvncserver.service
>  [Unit]
>  Description=TightVNC remote desktop server  
>  After=sshd.service
>  
>  [Service]
>  Type=dbus
>  ExecStart=/usr/bin/tightvncserver :1
>  User=pi
>  Type=forking
>  
>  [Install]
>  WantedBy=multi-user.target


- sudo chown root:root /etc/systemd/system/tightvncserver.service
- sudo chmod 755 /etc/systemd/system/tightvncserver.service
- sudo systemctl start  tightvncserver.service
- sudo systemctl status tightvncserver.service
- sudo systemctl enable tightvncserver.service

##  Setup serial port

- sudo systemctl disable hciuart
- sudo systemctl disable hciuart.service
- sudo systemctl disable bluetooth.service


- sudo systemctl stop    serial-getty@ttyS0.service
- sudo systemctl disable serial-getty@ttyS0.service
- sudo systemctl mask    serial-getty@ttyS0.service


- sudo usermod -a -G dialout pi
- sudo usermod -a -G tty pi


- sudo vim /boot/firmware/usercfg.txt
>  enable_uart=1
>  dtoverlay=disable-bt


- sudo vim /boot/firmware/cmdline.txt
>  Remove any "console=ttyAMA0,115200", if found

## Install ROS2

- sudo apt-get install locales
- sudo locale-gen en_US en_US.UTF-8
- sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
- export LANG=en_US.UTF-8
- locale


- sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
- echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


- sudo apt-get update
- sudo apt-get install python3-rospkg
- sudo apt-get install ros-galactic-desktop
- sudo apt-get install ros-galactic-teleop-twist-keyboard
- sudo apt-get install ros-galactic-tf2-tools ros-galactic-tf-transformations

- echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
- source ~/.bashrc
- rosversion  --distro
- ros2 doctor --report
- ros2 run demo_nodes_cpp talker   &
- ros2 run demo_nodes_cpp listener &
- killall talker listener

##  Setup & try LIDAR

- ls -l /dev | grep ttyUSB
- sudo chmod 666 /dev/ttyUSB0


- git clone https://github.com/Slamtec/rplidar_ros.git
- cd rplidar_ros
- git checkout ros2
- colcon build --symlink-install
- source install/setup.bash
- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser &
- ros2 launch rplidar_ros view_rplidar.launch.py &
- rviz2 -d rviz/rplidar_ros2.rviz

##  Try & calibrate MPU9250 IMU

- i2cdetect -y 1

- cd tools
- python3 calibrate_imu.py


## Install Navigation2 (Nav2)

- sudo apt-get install ros-galactic-navigation2
- sudo apt-get install ros-galactic-nav2-bringup
- sudo apt-get install ros-galactic-slam-toolbox
- sudo apt-get install ros-galactic-robot-localization


- sudo apt-get install ros-galactic-joint-state-publisher-gui
- sudo apt-get install ros-galactic-rqt-robot-steering
- sudo apt-get install ros-galactic-xacro


- ros2 launch rcap rcap.vm.display.launch.py
- rviz2 -d config.simulation.rviz
- ros2 launch slam_toolbox online_async_launch.py
- ros2 launch nav2_bringup navigation_launch.py
- ros2 launch nav2_bringup navigation_launch.py params_file:=/home/stef/RCAP/robot_car/config/nav2.simulation.yaml
- ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/local_costmap/voxel_grid visualization_marker:=/my_marker


- rqt_robot_steering --force-discover
- ros2 run teleop_twist_keyboard teleop_twist_keyboard
- ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' -1
- ros2 topic echo /goal_pose

## Commands found usefull while learning/testing

- ros2 run joint_state_publisher_gui joint_state_publisher_gui description/robot_car.urdf
- ros2 run tf2_ros static_transform_publisher 0 0 0 3.14 0 0 base_link lidar_link
- ros2 run teleop_twist_keyboard teleop_twist_keyboard

- rqt_graph
- ros2 run tf2_tools view_frames
- xdg-open frames.pdf 

- sudo service apport stop
