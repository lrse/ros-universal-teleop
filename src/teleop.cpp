#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <iostream>
#include "universal_teleop/Control.h"
#include "teleop.h"

using namespace std;
namespace teleop = universal_teleop;

teleop::Teleop::Teleop(void) : n("~"), key_override_enabled(false), joy_override_enabled(false)
{
  /* load mappings */
  if (n.hasParam("joy_axes")) n.getParam("joy_axes", joy_axes);
  else joy_axes = { {"pitch", 1}, {"roll", 0}, {"yaw", 3}, {"vertical",2} };
  for (auto& j : joy_axes) joy_axis_map[j.second] = j.first;

  map<string, int> joy_buttons;
  if (n.hasParam("joy_buttons")) n.getParam("joy_buttons", joy_buttons);
  else joy_buttons = { {"override", 4}, {"start", 2}, {"stop", 1}, {"takeoff", 10}, {"land", 11} };
  for (auto& j : joy_buttons) joy_button_map[j.second] = j.first;

  joy_deadzones = { { "pitch", 0.000001f }, { "roll", 0.000001f }, { "yaw", 0.000001f }, { "vertical", 0.000001f } };
  n.param("joy_deadzones", joy_deadzones, joy_deadzones);
  for (auto& k: joy_deadzones) cout << k.first << " " << k.second << endl;

  std::map<std::string, int> keys;
  if (n.hasParam("keys")) n.getParam("keys", keys);
  else keys = { {"override", 32}, {"start", 113}, {"stop", 97}, {"takeoff", 121}, {"land", 104} };
  for (auto& k : keys) key_map[k.second] = k.first;

  std::map<std::string, int> key_axes;
  if (n.hasParam("key_axes")) n.getParam("key_axes", key_axes);
  else key_axes = { { "pitch+", 273 }, { "pitch-", 274 }, { "yaw+", 276 }, { "yaw-", 275 } };
  for (auto& k : key_axes) key_axes_map[k.second] = k.first;
  key_axes_state = { { "pitch", 0 }, { "yaw", 0 } };

  axis_scales = { { "pitch", 1.0f }, { "roll", 1.0f }, { "yaw", 1.0f }, { "vertical", 1.0f } };
  n.param("scales", axis_scales, axis_scales);
  //for (auto& k: axis_scales) cout << k.first << " " << k.second << endl;

  n.param("send_velocity", send_velocity, true);

  /* subscribe to input sources */
  joy_sub = n.subscribe("/joy", 1, &Teleop::joystick_event, this);
  keyup_sub = n.subscribe("/keyboard/keyup", 1, &Teleop::keyboard_up_event, this);
  keydown_sub = n.subscribe("/keyboard/keydown", 1, &Teleop::keyboard_down_event, this);

  /* publish events and control commands */  
  pub_vel = n.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 5);
  
  pub_event = n.advertise<teleop::Event>("events", 5);
  pub_control = n.advertise<teleop::Control>("controls", 1);

  /* special events for UAV commands */
  pub_takeoff = n.advertise<std_msgs::Empty>("/robot/takeoff", 5);
  pub_land = n.advertise<std_msgs::Empty>("/robot/land", 5);
  pub_emergency = n.advertise<std_msgs::Empty>("/robot/reset", 5);
}

void teleop::Teleop::process_event(const teleop::Event& e)
{
  ROS_DEBUG_STREAM("event: " << e.event << " state: " << e.state);
  if (e.event == "override") {
    if (e.state == 0) {
      // when releasing override, stop robot 
      geometry_msgs::Twist vel;
      vel.linear.x = vel.linear.y = vel.linear.z = 0;
      vel.angular.x = vel.angular.y = 1; // non-zero to avoid hovering when zero-ing controls
      vel.angular.z = 0;
      pub_vel.publish(vel);
    }
  }
  else {
    if ((joy_override_enabled || key_override_enabled) && e.state) {
      if (e.event == "takeoff") pub_takeoff.publish(std_msgs::Empty());
      else if (e.event == "land") pub_land.publish(std_msgs::Empty());
      else if (e.event == "emergency") pub_emergency.publish(std_msgs::Empty());
    }
  }
  pub_event.publish(e);
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

      if (e.event == "override") joy_override_enabled = e.state;
      process_event(e);
    }
  }

  // process axis
  for (uint32_t a = 0; a < joy->axes.size(); a++) {
    if (joy->axes[a] != last_joy_msg.axes[a]) {
      teleop::Control c;
      if (joy_axis_map.find(a) == joy_axis_map.end()) c.control = "unknown";
      else c.control = joy_axis_map[a];

      if (c.control != "unknown" && std::abs(joy->axes[a]) < joy_deadzones[joy_axis_map[a]])
        c.value = 0;
      else {
        c.value = joy->axes[a];
      }

      pub_control.publish(c);
    }
  }
  
  last_joy_msg = *joy;
}

void teleop::Teleop::keyboard_up_event(const keyboard::Key::ConstPtr& key)
{
  ROS_DEBUG_STREAM("keyup: " << key->code);
  if (key_axes_map.find(key->code) != key_axes_map.end()) {
    std::string& cmd = key_axes_map[key->code];
    key_axes_state[cmd.substr(0, cmd.size() - 1)] = 0;
  }
  else {
    teleop::Event e;
    if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
    else e.event = key_map[key->code];
    e.state = 0;

    if (e.event == "override") key_override_enabled = false;
    process_event(e);
  }
}

void teleop::Teleop::keyboard_down_event(const keyboard::Key::ConstPtr& key)
{
  ROS_DEBUG_STREAM("keydown: " << key->code);

  if (key_axes_map.find(key->code) != key_axes_map.end()) {
    std::string& cmd = key_axes_map[key->code];
    key_axes_state[cmd.substr(0, cmd.size() - 1)] = (cmd[cmd.size() - 1] == '+' ? 1 : -1);
  }
  else {
    teleop::Event e;
    if (key_map.find(key->code) == key_map.end()) e.event = "unknown";
    else e.event = key_map[key->code];
    e.state = 1;

    if (e.event == "override") key_override_enabled = true;
    process_event(e);
  }
}

void teleop::Teleop::control(void)
{
  if (joy_override_enabled) {
    if (last_joy_msg.axes.empty()) return;

    float pitch = last_joy_msg.axes[joy_axes["pitch"]];
    float roll = last_joy_msg.axes[joy_axes["roll"]];
    float yaw = last_joy_msg.axes[joy_axes["yaw"]];
    float vertical = last_joy_msg.axes[joy_axes["vertical"]];

    /* check deadzones */
    if (std::abs(pitch) < joy_deadzones["pitch"]) pitch = 0;
    if (std::abs(yaw) < joy_deadzones["yaw"]) yaw = 0;
    if (std::abs(roll) < joy_deadzones["roll"]) roll = 0;
    if (std::abs(vertical) < joy_deadzones["vertical"]) vertical = 0;


    geometry_msgs::Twist vel;
    vel.linear.x = pitch * axis_scales["pitch"];
    vel.linear.y = roll * axis_scales["roll"];
    vel.linear.z = vertical * axis_scales["vertical"];
    vel.angular.x = vel.angular.y = 1; // non-zero to avoid hovering when zero-ing controls
    vel.angular.z = yaw * axis_scales["yaw"];
    pub_vel.publish(vel);
  }
  else if (key_override_enabled) {
    geometry_msgs::Twist vel;
    vel.linear.x = key_axes_state["pitch"] * axis_scales["pitch"];
    vel.linear.y = 0/*roll * axis_scales["roll"]*/;
    vel.linear.z = 0/*last_joy_msg.axes[joy_axes["vertical"]] * axis_scales["vertical"]*/;
    vel.angular.x = vel.angular.y = 1; // non-zero to avoid hovering when zero-ing controls
    vel.angular.z = key_axes_state["yaw"] * axis_scales["yaw"];
    pub_vel.publish(vel);
  }
}


