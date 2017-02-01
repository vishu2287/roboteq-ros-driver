# RoboteQ ROS Driver #

ROS driver for serial Roboteq motor controllers. This driver is suitable for use with Roboteq's Advanced Digital Motor Controllers, and it was implememted to be compatible with ROS Indigo Igloo distribution.

#Compatible Devices

* ax500
* ax1500
* ax2500
* ax2850
* ax3500


#Usage

The node subscribes to the folloing topics: 

```
Topic: roboteq_cmd_vel
Type: geometry_msgs::Twist
Contents:
	linear.x;
	linear.y;
	angular.z;
```

```
Topic: io_steer_angle
Type: geometry_msgs::Twist
Contents:
	angular.z;
```

The node publishes to the following topics:

```
Topic: roboteq_raw_vel
Type: geometry_msgs::Twist
Contents:
	linear.x
	linear.y
	angular.z
```

```
Topic: roboteq_estimated_pos
Type: geometry_msgs::Twist
Contents:
	linear.x
	linear.y
	angular.z
```

The ROSlaunch script includes all the configurable parameters.

#Credits

The code was ported from the Player - Stage Drivers from [here](http://sourceforge.net/projects/playerstage/files/Player/3.0.2/) and [here](https://github.com/uml-robotics/player-2.1.3/blob/master/server/drivers/position/roboteq/roboteq.cc).



