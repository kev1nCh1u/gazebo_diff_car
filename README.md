# gazebo_diff_car_ws

## gazebo models download

### reference
https://github.com/osrf/gazebo_models
https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
https://gist.github.com/awesomebytes/982166e825aa23ecdaf9acf34fa0a330

#### install
    mkdir .gazebo/models/
    https://github.com/kevin01yaya/gazebo_models

    cd ~/.gazebo/models/
    wget http://file.ncnynl.com/ros/gazebo_models.txt
    wget -i gazebo_models.txt
    ls model.tar.g* | xargs -n1 tar xzvf

## diff start
    roslaunch simulation_environment apartment.launch
    rosrun JoyStick joystick /dev/input/js0
    rosrun move_robot move_robot /dev/ttyUSB0 115200

## diff start all
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

## hector slam
    rostopic pub /Command hector_mapping/setmap_hec "{type: 'Create Map', Name: '', ini_pose_x: 0.0, ini_pose_y: 0.0, ini_pose_z: 0.0}"
    rostopic pub /Command hector_mapping/setmap_hec "{type: 'Save Map', Name: 'qq', ini_pose_x: 0.0, ini_pose_y: 0.0, ini_pose_z: 0.0}"
    rostopic pub /Command std_msgs/String "Load Map"
    rostopic pub /syscommand std_msgs/String "reset"
    rostopic pub /Command std_msgs/String "navigation=true"
    rostopic pub /Command d_msgs/String "navigation=false"
    rostopic pub /Command std_msgs/String "ReLoad Map"

## anhung task
    'Mr;0,0,-0.8,0.08,-0.02,diff,0,1.0,test;1,3,2.34,-2.21,1.53,diff,0,1.0,test,0.1;2,19,2.3,0.71,1.55,diff,0,1.0,test;3,3,3.02,-0.94,1.55,diff,0,1.0,test,2;4,3,2.98,1.89,-0.002,diff,0,1.0,test,0;5,3,3.97,1.82,-1.52,diff,0,1.0,test,0;6,3,4.09,-0.85,-1.52,diff,0,1.0,test,2;E'

    "Mr;1,0, -1.93,-0.44,-3.06,diff,0,0.5,ivam_3F;2,3, -1.94, -0.39,0.09,diff,0,0.5,ivam_3F,2;E"
