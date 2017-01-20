/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:imu_state.h
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_IMU_STATE_H_
#define PHODOM3_INCLUDE_IMU_STATE_H_

#include <eigen3/Eigen/Core>

class ImuState {
public:
	ImuState();
	void setImuStateVector(Eigen::VectorXd other);
	Eigen::VectorXd getImuStateVector();
	Eigen::Matrix3d T_g;
	Eigen::Matrix3d T_s;
	Eigen::Matrix3d T_a;
private:
	Eigen::VectorXd imuStateVector_;
};

#endif /* PHODOM3_INCLUDE_IMU_STATE_H_ */
