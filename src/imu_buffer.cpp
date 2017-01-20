/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:imu_buffer.cpp
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/
#include "imu_buffer.h"

ImuBuffer::ImuBuffer(int maxBufferLength) : data(maxBufferLength) {

}

ImuBuffer::~ImuBuffer() {

}

void ImuBuffer::print() {
    for(int ii = 0; ii < data.size(); ii++)
    {
    	std::cout << "time = " << data[ii].time << std::endl;
    	std::cout << data[ii].acceleration.transpose() << std::endl;
    	std::cout << data[ii].angularVelocity.transpose() << std::endl;
    }
}

void ImuBuffer::pruneBeforeTime(double time) {
	while(data.size()) {
		if(data.begin()->time < time) {
			data.pop_front();
		}
		else
			break;
	}
}

void ImuBuffer::shiftToInterpolationInterval(double time, boost::circular_buffer<ImuItem>::iterator &it) {
	int diff = it->time < time ? 1 : -1;
	while(it->time > time || time >= std::next(it)->time) {
		if(it == data.begin() && diff == -1) {
			break;
		}
		if(it+1 == data.end()-1 && diff == 1) {
			it--;
			break;
		}
		std::advance(it, diff);
	}
}

bool ImuBuffer::interpolate(double time, ImuItem &interpolated) {
	if(data.size() < 2 || time < data.front().time || time > data.back().time) {
		return false;
	}
	boost::circular_buffer<ImuItem>::iterator it = data.begin(); //shift from data.begin()
	shiftToInterpolationInterval(time, it);

	double delta_t = std::next(it)->time - it->time;
	Eigen::Vector3d k_a = (std::next(it)->acceleration - it->acceleration)/delta_t;
	Eigen::Vector3d k_w = (std::next(it)->angularVelocity - it->angularVelocity)/delta_t;

	interpolated.acceleration = it->acceleration - k_a*(it->time - time);
	interpolated.angularVelocity = it->angularVelocity - k_w*(it->time - time);
	interpolated.time = time;

	assert(interpolated.acceleration.norm() > 1e-100);
	assert(interpolated.acceleration.norm() < 1e100);
	assert(interpolated.angularVelocity.norm() > 1e-100);
	assert(interpolated.angularVelocity.norm() < 1e100);

	return true;
}
