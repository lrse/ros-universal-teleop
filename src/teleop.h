#ifndef __TELEOP_H__
#define __TELEOP_H__

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <keyboard/Key.h>

namespace universal_teleop {
  class Teleop {
    public:
      Teleop(void);
      void control(void);

    private:
      void joystick_event(const sensor_msgs::Joy::ConstPtr& joy);
      void keyboard_up_event(const keyboard::Key::ConstPtr& key);
      void keyboard_down_event(const keyboard::Key::ConstPtr& key);

      ros::NodeHandle n;
      
      ros::Subscriber joy_sub, keyup_sub, keydown_sub;
      ros::Publisher pub_vel, pub_event, pub_control;

      sensor_msgs::Joy last_joy_msg;
      std::map<int, std::string> joy_button_map;
      std::map<int, std::string> joy_axis_map;
      std::map<uint16_t, std::string> key_map;

      std::map<std::string, int> joy_axes;

      bool override_enabled;
  };
}

#endif
