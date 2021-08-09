# Axioma package test to connect to motors 

## Run

1. Setup environment variables:

        . ~/utadeo_ws/install/setup.bash

2. run the axioma node:

        ros2 lrun axioma_node axioma_node

3. other console (to command the robot)
        ros2 run rqt_robot_steering rqt_robot_steering

4. other console (to view the state of the motors)
        ros2  topic echo /joint_states
## Install

Assuming that the appropriate ROS2 (Foxy) version is already installed:

1. Install rthe required packages. 

        sudo apt install ros-foxy-rqt-robot-steering ros-foxy-robot-state-publisher ros-foxy-joy ros-foxy-joy-linux ros-foxy-joy-teleop
        pip3 install pyserial

3. Go to your colcon workspace scooby_description and scooby_package 
        cd ~catkin_ws/src


5. Clone axioma_node:

        git clone https://github.com/olmerg/axioma_robot.git

6. Build and install:

        cd ~/utadeo_ws
        colcon build

7. add serial conection to your user in linux (if it can not find serial port it will enter in simulation mode): dialout group to the user: 
        
        sudo usermod -a -G dialout name_user

8. check arduino mega is connected with the firmware:

        ls /dev/ttyACM*
  
## Robot with Arduino Mega:

Materials:

1. Arduino Mega 2560
2. low cost encoders (https://www.mactronica.com.co/sensor-encoder-optico-hc-020k) BAD IDEA
3. motor driver TB6612FNG  (https://www.mactronica.com.co/driver-tb6612fng-control-motor-paso-a-paso)
4. small car 2wd (https://www.mactronica.com.co/carro-robotico-smart-car-v2)



