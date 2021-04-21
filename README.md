# Gazebo sim car

## gazebo models download
https://github.com/osrf/gazebo_models

    ./gazebo_ws/src/download_gazebo_models/download.sh
    
## start
    roslaunch simulation_environment apartment.launch
    rosrun JoyStick joystick /dev/input/js0
    rosrun move_robot move_robot /dev/ttyUSB0 115200