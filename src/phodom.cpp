/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:phodom.cpp
*   Author  :linpenghong
*   Date    :Jan 17, 2017
*   Describe:TODO
*
********************************************************/
#include "phodom.h"

#include <string>

Phodom::Phodom() {
	imuBuffer_.reset(new ImuBuffer(100));
}

int Phodom::run()
{
	//---------load parameters from file---------//
	std::string parameter_file = "/home/lph/catkin_ws/src/phodom3/config/parameters.yaml";
	parameter_ = Parameter::fromPath(parameter_file);

	//---------ros subscriber and publisher---------//
    pathPub_  = nodeHandle_.advertise<nav_msgs::Path>("/path", 1);
    odomPub_  = nodeHandle_.advertise<nav_msgs::Odometry>("/odometry", 1);
//    imuSub_   = nodeHandle_.subscribe("/walle/sensors/imu", 50, &Phodom::imuCallback, this);
    imuSub_   = nodeHandle_.subscribe("/torso_lift_imu/data", 50, &Phodom::imuCallback, this);
    imageSub_ = nodeHandle_.subscribe("/wide_stereo/left/image_rect", 5, &Phodom::imageCallback, this);

    filter_.reset(new Filter(parameter_, imuBuffer_));

    ros::spin();
    return 0;
}

/*
 * imu callback, all message received would be pushed_back to the imu buffer
 */
void Phodom::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
	std::cout << "Received a imu message!" << std::endl;
	std::cout << "Reference state: " << msg->orientation.x << ", " << msg->orientation.y << ", " << msg->orientation.z << ", " << msg->orientation.w << std::endl;
	double time = getMessageTime(msg->header.stamp);
	ImuItem imu;
	imu.time = time;
	imu.acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	imu.angularVelocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	if(filter_->bodyState.is_initialized) {
		imuBuffer_->data.push_back(imu);
		filter_->propagateToTime(time);
		static nav_msgs::Odometry odom;
		odom.header.frame_id = "map";
		odom.header.stamp = msg->header.stamp;
		odom.pose.pose.position.x = filter_->bodyState.p_B_G(0);
		odom.pose.pose.position.y = filter_->bodyState.p_B_G(1);
		odom.pose.pose.position.z = filter_->bodyState.p_B_G(2);
//		odom.pose.pose.position.x = 0;
//		odom.pose.pose.position.y = 0;
//		odom.pose.pose.position.z = 0;

//		Eigen::Quaterniond odom_transform(0,1,0,0);//rotation around x axis for 180 degree
		Eigen::Quaterniond odom_transform(1,0,0,0);//unit quaternion
		Eigen::Quaterniond new_odom = odom_transform*filter_->bodyState.q_B_G;

		odom.pose.pose.orientation.w = new_odom.w();
		odom.pose.pose.orientation.x = new_odom.x();
		odom.pose.pose.orientation.y = new_odom.y();
		odom.pose.pose.orientation.z = new_odom.z();


		odom.twist.twist.linear.x = filter_->bodyState.v_B_G(0);
		odom.twist.twist.linear.y = filter_->bodyState.v_B_G(1);
		odom.twist.twist.linear.z = filter_->bodyState.v_B_G(2);
		odomPub_.publish(odom);

		static tf::TransformBroadcaster tfBr;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z));
		transform.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
		tfBr.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "phodom"));
	}
	else {
		static float counter = 0;
		static Eigen::Vector4d qq = Eigen::Vector4d::Zero();
		static Eigen::Vector3d gg = Eigen::Vector3d::Zero();
		static Eigen::Vector3d g_(0,0,-9.8);
		if(counter <= 9)
		{
			qq(0) += msg->orientation.w;
			qq(1) += msg->orientation.x;
			qq(2) += msg->orientation.y;
			qq(3) += msg->orientation.z;
			gg(0) += msg->linear_acceleration.x;
			gg(1) += msg->linear_acceleration.y;
			gg(2) += msg->linear_acceleration.z;
			counter++;
		}

		if(counter > 9)
		{
			Eigen::Quaterniond g2b;
			g2b.FromTwoVectors(g_, gg);

			filter_->bodyState.imu.time = time;
			filter_->bodyState.imu.acceleration.setZero();
			filter_->bodyState.imu.angularVelocity.setZero();
			filter_->bodyState.q_B_G = Eigen::Quaterniond(g2b.w(), g2b.x(), g2b.y(), g2b.z());
//			filter_->bodyState.q_B_G = Eigen::Quaterniond(qq(0)/10.0, qq(1)/10.0, qq(2)/10.0, qq(3)/10.0);
//			filter_->bodyState.q_B_G = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
			filter_->bodyState.s = filter_->bodyState.q_B_G.toRotationMatrix()*parameter_->getGlobalGravity()*0.01;//for 100 Hz imu frequency
			filter_->bodyState.y = filter_->bodyState.s * 0.01 * 0.5;
//			filter_->bodyState.q_B_G = Eigen::Quaterniond(1, 0, 0, 0);
			filter_->bodyState.is_initialized = true;
		}
	}
}

/*
 * image callback
 */
void Phodom::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	std::cout << "Received a image message!" << std::endl;
	double time = getMessageTime(msg->header.stamp);
	cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
	imageItem = ImageItem(time, image);
}

double Phodom::getMessageTime(ros::Time stamp) {
	static ros::Time beginTime = stamp;
	ros::Duration duration = stamp - beginTime;
	return duration.toSec();
}
