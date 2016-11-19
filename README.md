# Kobuki-Driver
Driver file for Kobuki robot
This is a kobuki driver program.

(1) launch the kobuki_node using:

	"roslaunch kobuki_node minimal.launch --screen"
(2) use
	
  "rostopic pub /mobile_base/commands/reset_odometry std_msgs/Empty"
  
to reset the position (position and angular equals to zero)
