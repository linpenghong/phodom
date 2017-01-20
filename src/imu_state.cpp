/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:imu_state.cpp
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/
#include "imu_state.h"
#include <iostream>

ImuState::ImuState() {
	T_g.setIdentity();
	T_s.setZero();
	T_a.setIdentity();
	imuStateVector_.setZero(27);
}

void ImuState::setImuStateVector(Eigen::VectorXd other) {
	imuStateVector_ = other;
}

Eigen::VectorXd ImuState::getImuStateVector() {
	for(int ii=0; ii < 9; ii++) {
		imuStateVector_(ii) = T_g(ii);
	}
	for(int ii=9; ii < 18; ii++) {
		imuStateVector_(ii) = T_s(ii-9);
	}
	for(int ii=18; ii < 27; ii++) {
		imuStateVector_(ii) = T_a(ii-18);
	}
	return imuStateVector_;
}
