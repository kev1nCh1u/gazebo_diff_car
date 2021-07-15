# gazebo_diff_car_ws

## gazebo models download

### reference
https://github.com/osrf/gazebo_models
https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
https://gist.github.com/awesomebytes/982166e825aa23ecdaf9acf34fa0a330

#### install
    mkdir .gazebo/models/
    ./download_gazebo_models/download.sh

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

## install
    sudo apt install cppad
    sudo apt-get install ros-<version>-ifopt

### Ipopt
    sudo apt-get install gfortran
    sudo apt-get install unzip
    wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
    sudo ./install_ipopt.sh Ipopt-3.12.7
