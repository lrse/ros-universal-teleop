= Description =

The purpose of this node is to allow a transparent layer for tele-operating robots, re-configurable and which does not depend on a particular robot. Both terrestrial and aerial robots are supported.
This node reads input from keyboard (using the corresponding node) and joystick. Button and axis can be associated to "actions" and axis controls. The output is a geometry_msgs/Twist message, which is standard
for controlling robots. The node will only emit velocity commands when the "override" is enabled (a special event) by the corresponding button/key. This allows the universal_teleop to co-exist with other
nodes which control a robot autonomously. Whenever the "override" event is fired, this event can be received from any other node, disabling autonomous control and letting universal_teleop to control the robot.
Furthermore, the requirement for this "override" to manually control the robot functions as a dead man's switch. If the joystick falls to the floor, the override will be released and the robot will be stopped.

= Topics =

== Subscribed ==

/joy (joy/Joy): input from joystick, reading buttons and axis

/keyboard/keyup, /keyboard/keydown (keyboard/Key): input from keyboard, key-up and key-down events respectively

== Advertised ==

/robot/cmd_vel (geometry_msgs/Twist): output velocity commands to robot

~events (universal_teleop/Event): output events

~controls (universal_teleop/Control): output controls

/robot/takeoff, /robot/land, /robot/reset: special commands used on aerial-robots for controlling takeoff and landings
