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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

Phodom::Phodom() {
	imuBuffer_.reset(new ImuBuffer(100));
}

int Phodom::run()
{
	//---------load parameters from file---------//
	std::string parameter_file = "/home/lph/catkin_ws/src/phodom3/config/parameters.yaml";
	parameter_ = Parameter::fromPath(parameter_file);

    tf_buffer_.reset(new tf2_ros::Buffer);
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

	//---------ros subscriber and publisher---------//
    pathPub_  = nodeHandle_.advertise<nav_msgs::Path>("/path", 1);
    odomPub_  = nodeHandle_.advertise<nav_msgs::Odometry>("/odometry", 1);


//    imuSub_   = nodeHandle_.subscribe("/kitti/oxts/imu", 50, &Phodom::imuCallback, this);
//    imageSub_ = nodeHandle_.subscribe("/kitti/camera_color_right/image_raw", 5, &Phodom::imageCallback, this);


    sleep(5);//waiting for tf msg

    Eigen::Quaterniond q_C_B = Eigen::Quaterniond::Identity();
    Eigen::Vector3d p_C_B;
    try {
        geometry_msgs::TransformStamped body_to_camera_transform = tf_buffer_->lookupTransform("wide_stereo_optical_frame", "imu_link", ros::Time(0));
        double x = body_to_camera_transform.transform.rotation.x;
        double y = body_to_camera_transform.transform.rotation.y;
        double z = body_to_camera_transform.transform.rotation.z;
        double w = body_to_camera_transform.transform.rotation.w;
        q_C_B = Eigen::Quaterniond(w, x, y, z);

        tf::vectorMsgToEigen(body_to_camera_transform.transform.translation, p_C_B);
    } catch (tf2::TransformException& e) {
        ROS_ERROR("%s", e.what());
        return 1;
    }

    imuSub_   = nodeHandle_.subscribe("/torso_lift_imu/data", 50, &Phodom::imuCallback, this);
    imageSub_ = nodeHandle_.subscribe("/wide_stereo/left/image_rect", 5, &Phodom::imageCallback, this);

    filter_.reset(new Filter(parameter_, imuBuffer_));
    filter_->parameter->setBodyToCameraRotation(q_C_B);
    filter_->position_of_body_in_camera_ = -p_C_B;
    ros::spin();
    return 0;
}


/*
 * imu callback, all message received would be pushed_back to the imu buffer
 */
void Phodom::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
//	std::cout << "Received a imu message!" << std::endl;
//	std::cout << "Reference state: " << msg->orientation.x << ", " << msg->orientation.y << ", " << msg->orientation.z << ", " << msg->orientation.w << std::endl;

	double time = getMessageTime(msg->header.stamp);
	ImuItem imu;
	imu.time = time;
	imu.acceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	imu.angularVelocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	if(filter_->bodyState.is_initialized) {
		imuBuffer_->data.push_back(imu);
		filter_->propagateToTime(time);
		static nav_msgs::Odometry odom;
		odom.header.frame_id = "imu_link";
		odom.header.stamp = msg->header.stamp;
		odom.pose.pose.position.x = filter_->bodyState.p_B_G(0);
		odom.pose.pose.position.y = filter_->bodyState.p_B_G(1);
		odom.pose.pose.position.z = filter_->bodyState.p_B_G(2);
//		odom.pose.pose.position.x = 0;
//		odom.pose.pose.position.y = 0;
//		odom.pose.pose.position.z = 0;

		Eigen::Quaterniond odom_transform(0,1,0,0);//rotation around x axis for 180 degree
//		Eigen::Quaterniond odom_transform(1,0,0,0);//unit quaternion
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
			//filter initialise
			Eigen::Quaterniond g2b;
			g2b = Rotation::fromTwoVectors(g_, gg/10.0);

			filter_->optical_center_ = parameter_->getCameraOpticalCenter();
			filter_->focal_point_ = parameter_->getCameraFocalPoint();
			filter_->camera_delay_ = parameter_->getCameraDelayTime();
			filter_->camera_readout_ = parameter_->getCameraReadoutTime();
			filter_->radial_distortion_ = parameter_->getCameraRadialDistortionParams();
			filter_->tangential_distortion_ = parameter_->getCameraTangentialDistortionParams();

//			geometry_msgs::TransformStamped map_to_body_transform = tf_buffer_->lookupTransform("imu_link", "map", ros::Time(0));
//			double x = map_to_body_transform.transform.rotation.x;
//			double y = map_to_body_transform.transform.rotation.y;
//			double z = map_to_body_transform.transform.rotation.z;
//			double w = map_to_body_transform.transform.rotation.w;
//			Eigen::Quaterniond q_B_G(w, x, y, z);
//			Eigen::Vector3d p_B_G;
//			tf::vectorMsgToEigen(map_to_body_transform.transform.translation, p_B_G);
//			Eigen::Quaterniond odom_transform(-0.05,0.998749, -0.1, 0);
//			odom_transform.normalize();
//			filter_->bodyState.p_B_G = p_B_G;
//			filter_->bodyState.q_B_G = odom_transform*q_B_G;


			filter_->bodyState.imu.time = time;
			filter_->bodyState.imu.acceleration.setZero();
			filter_->bodyState.imu.angularVelocity.setZero();
			filter_->bodyState.q_B_G = Eigen::Quaterniond(g2b.w(), g2b.x(), g2b.y(), g2b.z()); //quaternion from two vector need to reconstruct
//			filter_->bodyState.q_B_G = Eigen::Quaterniond(qq(0)/10.0, qq(1)/10.0, qq(2)/10.0, qq(3)/10.0);
//			filter_->bodyState.q_B_G = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
			filter_->bodyState.s = filter_->bodyState.q_B_G.toRotationMatrix()*parameter_->getGlobalGravity()*0.01;//for 100 Hz imu frequency
			filter_->bodyState.y = filter_->bodyState.s * 0.01 * 0.5;

			filter_->imuState.T_a = parameter_->getAccelerometerShapeMatrix();
			filter_->imuState.T_g = parameter_->getGyroscopeShapeMatrix();
			filter_->imuState.T_s = parameter_->getGyroscopeAccelerationSensitivityMatrix();

		    Eigen::VectorXd covar_diag(56);
		    covar_diag.segment<3>(0) = parameter_->getOrientationNoise();
		    covar_diag.segment<3>(3) = parameter_->getPositionNoise();
		    covar_diag.segment<3>(6) = parameter_->getVelocityNoise();
		    covar_diag.segment<3>(9) = parameter_->getGyroscopeBiasNoise();
		    covar_diag.segment<3>(12) = parameter_->getAccelerometerBiasNoise();
		    covar_diag.segment<3>(15) = parameter_->getGyroscopeShapeMatrixNoise().block<1, 3>(0, 0).transpose();
		    covar_diag.segment<3>(18) = parameter_->getGyroscopeShapeMatrixNoise().block<1, 3>(1, 0).transpose();
		    covar_diag.segment<3>(21) = parameter_->getGyroscopeShapeMatrixNoise().block<1, 3>(2, 0).transpose();
		    covar_diag.segment<3>(24) = parameter_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(0, 0).transpose();
		    covar_diag.segment<3>(27) = parameter_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(1, 0).transpose();
		    covar_diag.segment<3>(30) = parameter_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(2, 0).transpose();
		    covar_diag.segment<3>(33) = parameter_->getAccelerometerShapeMatrixNoise().block<1, 3>(0, 0).transpose();
		    covar_diag.segment<3>(36) = parameter_->getAccelerometerShapeMatrixNoise().block<1, 3>(1, 0).transpose();
		    covar_diag.segment<3>(39) = parameter_->getAccelerometerShapeMatrixNoise().block<1, 3>(2, 0).transpose();
		    covar_diag.segment<3>(42) = parameter_->getPositionOfBodyInCameraFrameNoise();
		    covar_diag.segment<2>(45) = parameter_->getFocalPointNoise();
		    covar_diag.segment<2>(47) = parameter_->getOpticalCenterNoise();
		    covar_diag.segment<3>(49) = parameter_->getRadialDistortionNoise();
		    covar_diag.segment<2>(52) = parameter_->getTangentialDistortionNoise();
		    covar_diag(54) = parameter_->getCameraDelayTimeNoise();
		    covar_diag(55) = parameter_->getCameraReadoutTimeNoise();

			filter_->bodyState.covariance.setZero();
			filter_->bodyState.covariance = covar_diag.asDiagonal();

			filter_->bodyState.is_initialized = true;
		}
	}
}

/*
 * image callback
 */
void Phodom::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	std::cout << "=============Received a image message!==============" << std::endl;
	std::cout << "image frame id = " << msg->header.frame_id << std::endl;
//	std::cout << msg->width << ", " << msg->height << std::endl;
	double time = getMessageTime(msg->header.stamp);
	cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
	imageItem = ImageItem(time, image);

	//force the body state be the same with tf
//	geometry_msgs::TransformStamped map_to_body_transform = tf_buffer_->lookupTransform("map", "imu_link", msg->header.stamp);
//	double x = map_to_body_transform.transform.rotation.x;
//	double y = map_to_body_transform.transform.rotation.y;
//	double z = map_to_body_transform.transform.rotation.z;
//	double w = map_to_body_transform.transform.rotation.w;
//	Eigen::Quaterniond q_B_G(w, x, y, z);
//	Eigen::Vector3d p_B_G;
//	tf::vectorMsgToEigen(map_to_body_transform.transform.translation, p_B_G);
//	Eigen::Quaterniond odom_transform(0,1,0,0);
//	filter_->bodyState.p_B_G = p_B_G;
//	filter_->bodyState.q_B_G = odom_transform*q_B_G;
//	filter_->bodyState.v_B_G.setZero();

	filter_->stepImage(imageItem.getTime(), imageItem.getImage(), imuIter_);

//    cv::Ptr<cv::FeatureDetector> detector_;
//    cv::Ptr<cv::DescriptorExtractor> extractor_;
//    detector_ = cv::FeatureDetector::create("ORB");
//    detector_->set("nFeatures", 100);
//    extractor_ = cv::DescriptorExtractor::create("ORB");
//	Feature feature = Feature::detectFeatures(detector_, extractor_, imageItem.getImage());
}

double Phodom::getMessageTime(ros::Time stamp) {
	static ros::Time beginTime = stamp;
	ros::Duration duration = stamp - beginTime;
	return duration.toSec();
}
