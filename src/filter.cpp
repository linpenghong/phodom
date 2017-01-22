/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:filter.cpp
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/
#include "filter.h"
#include "parameter.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>

Filter::Filter(std::shared_ptr<Parameter> parameter_, std::shared_ptr<ImuBuffer> imuBuffer_) : bodyState(parameter_) {
	parameter = parameter_;
	imuBuffer = imuBuffer_;
	propaState.setZero(57);
	augmentState.setZero(57);
	propaCovariance.setZero(56,56);
	augmentCovariance.setZero(56,56);
}

void Filter::computeImuEstimate(boost::circular_buffer<ImuItem>::iterator it) {
	imuEstimate_.time = it->time;
	imuEstimate_.acceleration = imuState.T_a.inverse() * (it->acceleration - bodyState.b_a);
	imuEstimate_.angularVelocity = imuState.T_g.inverse() * (it->angularVelocity - imuState.T_s*imuEstimate_.acceleration - bodyState.b_g);
	assert(imuEstimate_.acceleration.norm() < 1000);
	assert(imuEstimate_.angularVelocity.norm() < 1000);
}

void Filter::propagateToTime(double time) {
	assert(!imuBuffer->data.empty());
//	assert(imuBuffer->data.begin()->time < time);

//	imuBuffer->print();

	while(!imuBuffer->data.empty() && imuBuffer->data.begin()->time < time) {
		computeImuEstimate(imuBuffer->data.begin());
		BodyState nextState(bodyState);
		bodyState.propagateBodyState(nextState, imuEstimate_);
		bodyState.propagateCovariance(*this, nextState);
		bodyState = nextState;
		imuBuffer->data.pop_front();
	}
	std::cout << "New Body State : " << bodyState.q_B_G.x() << ", " << bodyState.q_B_G.y() << ", " << bodyState.q_B_G.z() << ", " << bodyState.q_B_G.w() << std::endl;
	std::cout << "New Covariance :\n" << bodyState.covariance << std::endl;
}
