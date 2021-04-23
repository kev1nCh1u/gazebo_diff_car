# kevin_cartographer_ws


## kevin 2d
    roslaunch urg_node urg_lidar.launch
    roslaunch launch_start my_robot_2d.launch

## kevin 3d
    roslaunch velodyne_pointcloud VLP16_points.launch

    roslaunch kevin_vmu931 imu_read_all.launch
    roslaunch imu_launch imu_msg.launch 

    roslaunch launch_start my_robot_3d.launch

## create map
    roslaunch launch_start hokuyo_2d.launch

## load map
    roslaunch launch_start tutorial.launch

    roslaunch launch_start my_robot_3d_load.launch
## cartographer build
    catkin_make_isolated --install --use-ninja
    source devel_isolated/setup.bash
## 存地圖指令
    rosservice call /finish_trajectory 0

    rosservice call /write_state "{filename: '/home/user/ros/kevin_cartographer_ws/src/ros_map/maps/mymap.pbstream'}"

## pbstream to rosmap
    rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/user/ros/kevin_cartographer_ws/src/ros_map/maps/mymap -pbstream_filename=/home/user/ros/kevin_cartographer_ws/src/ros_map/maps/mymap.pbstream -resolution=0.05

## ply point cloud
http://lidarview.com/

    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=${HOME}/Downloads/b3-2016-02-02-13-32-01.bag

    roslaunch launch_start assets_writer_backpack_3d.launch \
    bag_filenames:=${HOME}/Downloads/b3-2016-02-02-13-32-01.bag \
    pose_graph_filename:=${HOME}/Downloads/b3-2016-02-02-13-32-01.bag.pbstream

    roslaunch launch_start assets_writer_backpack_3d.launch \
    bag_filenames:=/home/user/ros/kevin_cartographer_ws/2021-03-15-18-23-49.bag \
    pose_graph_filename:=/home/user/ros/kevin_cartographer_ws/src/ros_map/maps/mymap.pbstream

    rosrun kevin_ply ply_publisher

## offline
    roslaunch cartographer_ros offline_backpack_3d.launch bag_filenames:=/home/user/rosbags/cartographer/b3-2016-04-05-13-54-42.bag

offline create map

    roslaunch launch_start offline_backpack_3d.launch bag_filenames:=/home/user/ros/kevin_cartographer_ws/2021-03-15-18-23-49.bag
## velodyne

    ip 192.168.1.201

    roslaunch velodyne_pointcloud VLP16_points.launch

    rosrun rviz rviz -f velodyne

    rosrun rviz rviz -d /home/user/ros/kevin_cartographer_ws/src/vlp_20210223.rviz