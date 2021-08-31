#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <windows.h>
#endif

//kevin
#include "turtlesim/joystick.h"
#include <boost/thread.hpp>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
      : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char *c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for (;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if (events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }

private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

  //kevin
  ros::Publisher joystick_pub_;
  void PubLoop(double period);

private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;

  //kevin
  int btn_id;
  boost::thread* receive_thread_;
};

TeleopTurtle::TeleopTurtle() : linear_(0),
                               angular_(0),
                               l_scale_(1.0),
                               a_scale_(1.0),
                               btn_id(0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  
  //kevin
  joystick_pub_ = nh_.advertise<turtlesim::joystick>("joystick", 1);
  receive_thread_ = new boost::thread(boost::bind(&TeleopTurtle::PubLoop, this, 0.01));
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

//kevin
turtlesim::joystick joystick_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT, quit);

  teleop_turtle.keyLoop();
  quit(0);

  return (0);
}

void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty = false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    // linear_ = angular_ = 0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch (c)
    {
    case KEYCODE_LEFT:
      ROS_DEBUG("LEFT");
      angular_ += 1.0;
      dirty = true;
      break;
    case KEYCODE_RIGHT:
      ROS_DEBUG("RIGHT");
      angular_ += -1.0;
      dirty = true;
      break;
    case KEYCODE_UP:
      ROS_DEBUG("UP");
      linear_ += 1.0;
      dirty = true;
      break;
    case KEYCODE_DOWN:
      ROS_DEBUG("DOWN");
      linear_ += -1.0;
      dirty = true;
      break;
    case KEYCODE_Q:
      ROS_DEBUG("quit");
      return;

    // kevin
    case KEYCODE_B:
      ROS_DEBUG("B");
      btn_id = 0;
      dirty = true;
      break;
    case KEYCODE_C:
      ROS_DEBUG("C");
      btn_id = 5;
      dirty = true;
      break;
    case KEYCODE_D:
      ROS_DEBUG("zero");
      btn_id = 0;
      angular_ = 0.0;
      linear_ = 0.0;
      dirty = true;
      break;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * angular_;
    twist.linear.x = l_scale_ * linear_;

    //kevin
    joystick_msg.btn_id = btn_id;
    joystick_msg.x = l_scale_ * linear_;
    joystick_msg.y = a_scale_ * angular_;
    joystick_msg.z = 0.0;

    if (dirty == true)
    {
      // twist_pub_.publish(twist);
      dirty = false;
    }
  }

  return;
}

void TeleopTurtle::PubLoop(double period)
{
  ros::Rate r_receive(1.0 / period);
  while (1)
  {
    std::cout << "btn_id:" << std::to_string(joystick_msg.btn_id) << " x:" << joystick_msg.x << " y:" << joystick_msg.y << "\n";
    joystick_pub_.publish(joystick_msg); //kevin
    r_receive.sleep();
  }
}
