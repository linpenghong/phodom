/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:ros_test.cpp
*   Author  :linpenghong
*   Date    :Jan 17, 2017
*   Describe:TODO
*
********************************************************/
#include "phodom.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char* argv[])
{
	std::cout << "========>In odom3 ros test!<========" << std::endl;
	ros::init(argc, argv, "phodom_ros");
	Phodom myodom;
	myodom.run();
	return 0;
}
