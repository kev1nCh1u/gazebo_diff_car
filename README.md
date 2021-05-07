# gazebo_diff_car_ws

## gazebo models download
https://github.com/osrf/gazebo_models

    ./gazebo_ws/src/download_gazebo_models/download.sh

## diff start
    roslaunch simulation_environment apartment.launch
    rosrun JoyStick joystick /dev/input/js0
    rosrun move_robot move_robot /dev/ttyUSB0 115200

## diff launch_start
    roslaunch launch_start start.launch

## create map
    roslaunch kevin_cartographer hokuyo_2d.launch

## gazebo velodyne example
roslaunch velodyne_description example.launch

## reference
https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/