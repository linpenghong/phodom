/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:filter.h
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_FILTER_H_
#define PHODOM3_INCLUDE_FILTER_H_

#include "imu_buffer.h"
#include "image_item.h"
#include "parameter.h"
#include "rotation.h"
#include "body_state.h"
#include "imu_state.h"
#include "parameter_state.h"

#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include <iostream>

class Filter {
public:
	Filter(std::shared_ptr<Parameter> parameter_, std::shared_ptr<ImuBuffer> imuBuffer_);
	void initialize();
	void reset();
	void computeImuEstimate(boost::circular_buffer<ImuItem>::iterator it);
	void propagateToTime(double time);
	void stateAugment();
	void update();
public:
	Eigen::VectorXd propaState;
	Eigen::VectorXd augmentState;
	Eigen::MatrixXd propaCovariance;
	Eigen::MatrixXd augmentCovariance;
	Eigen::Matrix3d R_B_G;

	BodyState bodyState;
	ImuState imuState;
	ParameterState paramState;

	std::shared_ptr<Parameter> parameter;
	std::shared_ptr<ImuBuffer> imuBuffer;
private:
	void propagateIMUStateAndCovar();

private:
	ImuItem imuEstimate_;
};

#endif /* PHODOM3_INCLUDE_FILTER_H_ */
