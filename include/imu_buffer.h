/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:imu_buffer.h
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM2_INCLUDE_IMU_BUFFER_H_
#define PHODOM2_INCLUDE_IMU_BUFFER_H_

#include <boost/circular_buffer.hpp>
#include <iterator>
#include <eigen3/Eigen/Core>
#include <iostream>

struct ImuItem{
	double time;
	Eigen::Vector3d acceleration;
	Eigen::Vector3d angularVelocity;
};

class ImuBuffer {
public:
	using container_type = boost::circular_buffer<ImuItem>;
	using iterator = container_type::iterator;
	using const_iterator = container_type::const_iterator;

	ImuBuffer(int maxBufferLength);
	~ImuBuffer();
	bool interpolate(double time, ImuItem &interpolated);//linear interpolation, return true if interpolated
	void pruneBeforeTime(double time);

	container_type data;

	//TODO:remove print function and iostream head include
	void print();
private:
	void shiftToInterpolationInterval(double time, boost::circular_buffer<ImuItem>::iterator &it);
};

#endif /* PHODOM2_INCLUDE_IMU_BUFFER_H_ */
