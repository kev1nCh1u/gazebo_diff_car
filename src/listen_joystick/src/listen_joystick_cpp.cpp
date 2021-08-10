#include "ros/ros.h"
#include "std_msgs/String.h"
#include "JoyStick/joystick.h"

void chatterCallback(const JoyStick::joystick::ConstPtr& msg)
{
  ROS_INFO("x: [%f]", msg->x);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listen_joystick_cpp");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("joystick", 1000, chatterCallback);

  ros::spin();

  return 0;
}