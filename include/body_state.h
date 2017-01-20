/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:body_state.h
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_BODY_STATE_H_
#define PHODOM3_INCLUDE_BODY_STATE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "imu_buffer.h"
#include "parameter.h"

class BodyState {
public:
	BodyState(std::shared_ptr<const Parameter> parameter);
	~BodyState();
	Eigen::VectorXd getBodyStateVector();
	void setBodyStateVector(Eigen::VectorXd other);
	void propagateBodyState(BodyState &state_l1, const ImuItem &imu_l1);
	Eigen::Matrix<double, 56, 56> propagateCovariance(BodyState &state_l1);


	BodyState& operator=(const BodyState& other) = default;
    BodyState& operator=(BodyState&& other) = default;
    BodyState(const BodyState& other) = default;
    BodyState(BodyState& other) = default;

public:
    bool is_initialized;
	ImuItem imu;
	Eigen::Quaterniond q_B_G;
	Eigen::Vector3d p_B_G;
	Eigen::Vector3d v_B_G;
	Eigen::Vector3d b_g;
	Eigen::Vector3d b_a;
	Eigen::Vector3d s;
	Eigen::Vector3d y;
	Eigen::Matrix<double, 56, 56> covariance;
private:
	Eigen::Matrix<double, 15, 15> getBodyStateTransitionMatrix(BodyState &state_l1);
	Eigen::Matrix<double, 15, 27> getImuCalibrationParamsTransitionMatrix(BodyState &state_l1);
	Eigen::Matrix<double, 15, 15> propagationNoiseMatrix(BodyState &state_l1);
	Eigen::VectorXd bodyStateVector_;
	std::shared_ptr<const Parameter> parameter_;
};

#endif /* PHODOM3_INCLUDE_BODY_STATE_H_ */
