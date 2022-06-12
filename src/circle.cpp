#include "two_drones/gnc_functions.hpp"

#include <ros/ros.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "fly_in_circle");
	ros::NodeHandle nh("~");
	std::string ros_namespace;
	nh.param<std::string>("nampespace", ros_namespace, "drone1");
	if (!nh.getParam("ros_namespace", ros_namespace))
		{ ROS_ERROR("No namespace param"); 
		  ROS_INFO_STREAM("using default namespace: drone1" );
		  
		}
	
	init_publisher_subscriber(nh, ros_namespace);

	wait4connect();

	set_mode("GUIDED");

	initialize_local_frame();

	takeoff(3);

	ros::Rate loop_rate(10);
	float x=5, y=0, z=3, theta = 0;

	geometry_msgs::Point location;

	while (ros::ok())
	{
		location = get_current_location();

		theta = atan(location.y/location.z);

		theta += 0.01;

		x = 5*cos(theta);
		y = 5*sin(theta);

		set_destination(x, y, z, 0);
	
		ros::spinOnce();
		loop_rate.sleep();
	}
}


