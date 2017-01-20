/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:phodom.h
*   Author  :linpenghong
*   Date    :Jan 17, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_PHODOM_H_
#define PHODOM3_INCLUDE_PHODOM_H_

#include "imu_buffer.h"
#include "parameter.h"
#include "body_state.h"
#include "imu_state.h"
#include "filter.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Core>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class ImuBuffer;
class Parameter;

class Phodom {
public:
	Phodom();
	int run();

private:
	void imuCallback(const sensor_msgs::ImuConstPtr& msg);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	double getMessageTime(ros::Time stamp);
	//=================ROS===================//
	ros::NodeHandle nodeHandle_;
    nav_msgs::Path path_;
    nav_msgs::Odometry odom_;
    ros::Publisher pathPub_;
    ros::Publisher odomPub_;
    ros::Subscriber imuSub_;
    ros::Subscriber imageSub_;

    //==============variables================//
    std::shared_ptr<ImuBuffer> imuBuffer_;
    std::shared_ptr<Parameter> parameter_;
    std::shared_ptr<Filter> filter_;
    ImageItem imageItem;
};

#endif /* PHODOM3_INCLUDE_PHODOM_H_ */
