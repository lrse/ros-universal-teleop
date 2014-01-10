#include <ros/ros.h>
#include "teleop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_teleop");

  universal_teleop::Teleop t;

  ros::Rate r(20);
  while(ros::ok()) {
    ros::spinOnce();
    t.control();
    r.sleep();
  }
}
