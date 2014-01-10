#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include "universal_teleop/Event.h"
#include "universal_teleop/Control.h"
#include "teleop.h"

using namespace std;
namespace teleop = universal_teleop;

teleop::Teleop::Teleop(void) : n("~"), override_enabled(false)
{
  /* load mappings */
  joy_axes = { {"pitch", 1}, {"roll", 3}, {"yaw", 0}, {"vertical",4} };
  n.param("joy_axes", joy_axes, joy_axes);
  for (auto& j : joy_axes) joy_axis_map[j.second] = j.first;
  
  map<string, int> joy_buttons = { {"override", 4}, {"start", 0}, {"stop", 1}, {"takeoff", 2}, {"land", 3} };
  n.param("joy_buttons", joy_buttons, joy_buttons);
  for (auto& j : joy_buttons) joy_button_map[j.second] = j.first;

  std::map<std::string, int> keys;
  n.param("keys", keys, keys);
  for (auto& k : keys) key_map[k.second] = k.first;

  /* subscribe to input sources */
  joy_sub = n.subscribe("/joy", 1, &Teleop::joystick_event, this);
  keyup_sub = n.subscribe("/keyboard/keyup", 1, &Teleop::keyboard_up_event, this);
  keydown_sub = n.subscribe("/keyboard/keydown", 1, &Teleop::keyboard_down_event, this);

  /* publish events and control commands */  
  pub_vel = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 5);
  
  pub_event = n.advertise<teleop::Event>("events", 5);
  pub_control = n.advertise<teleop::Control>("controls", 1);
}

void teleop::Teleop::joystick_event(const sensor_msgs::Joy::ConstPtr& joy)
{
  if (last_joy_msg.axes.empty()) {
    last_joy_msg = *joy;
    for (auto& b : last_joy_msg.buttons) b = 0;
  }

  // process buttons
  for (uint32_t b = 0; b < joy->buttons.size(); b++) {
    if (joy->buttons[b] != last_joy_msg.buttons[b]) {
      teleop::Event e;
      if (joy_button_map.find(b) == joy_button_map.end()) e.event = "unknown";
      else e.event = joy_button_map[b];
      e.state = joy->buttons[b];      
      pub_event.publish(e);
      if (e.event == "override") override_enabled = e.state;
    }
  }

  // process axis
  for (uint32_t a = 0; a < joy->axes.size(); a++) {
    if (joy->axes[a] != last_joy_msg.axes[a]) {
      teleop::Control c;
      if (joy_axis_map.find(a) == joy_axis_map.end()) c.control = "unknown";
      else c.control = joy_axis_map[a];
      c.value = joy->axes[a];      
      pub_control.publish(c);
    }
  }
  
  last_joy_msg = *joy;
}

void teleop::Teleop::keyboard_up_event(const keyboard::Key::ConstPtr& key)
{
  ROS_INFO_STREAM("keyup: " << key->code);
  teleop::Event e;
  if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
  else e.event = key_map[key->code];
  e.state = 0;
  pub_event.publish(e);
}

void teleop::Teleop::keyboard_down_event(const keyboard::Key::ConstPtr& key)
{
  teleop::Event e;
  if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
  else e.event = key_map[key->code];
  e.state = 1;
  pub_event.publish(e);
}

void teleop::Teleop::control(void)
{
  if (override_enabled) {
    geometry_msgs::Twist vel;
    vel.linear.x = last_joy_msg.axes[joy_axes["pitch"]];
    vel.linear.y = last_joy_msg.axes[joy_axes["roll"]];
    vel.linear.z = last_joy_msg.axes[joy_axes["vertical"]];
    vel.angular.x = vel.angular.y = 0;
    vel.angular.z = last_joy_msg.axes[joy_axes["yaw"]];
    pub_vel.publish(vel);
  }
}


