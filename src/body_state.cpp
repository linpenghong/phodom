/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:body_state.cpp
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/
#include "body_state.h"
#include "rotation.h"
#include "parameter.h"
#include <iostream>

BodyState::BodyState(std::shared_ptr<const Parameter> parameter) {
	q_B_G.setIdentity();
	p_B_G.setZero();
	v_B_G.setZero();
	b_g.setZero();
	b_a.setZero();
	bodyStateVector_.setZero(16);
	parameter_ = parameter;
	is_initialized = false;
}

BodyState::~BodyState() {

}

void BodyState::setBodyStateVector(Eigen::VectorXd other) {
	bodyStateVector_ = other;
}

/*
 * TODO: imu 直接积分误差很大，可以考虑propagation用互补滤波？
 */
void BodyState::propagateBodyState(BodyState &state_l1, const ImuItem &imu_l1) {
	assert(imu_l1.time > imu.time);
	double delta_t = imu_l1.time - imu.time;

	//gyroscope measurements
    Eigen::Vector4d q0(0.0, 0.0, 0.0, 1.0);
	Eigen::Vector4d k1 = 0.5 * Rotation::bigOmegaMatrix(imu.angularVelocity)*q0;
	Eigen::Vector4d k2 = 0.5 * Rotation::bigOmegaMatrix((imu_l1.angularVelocity + imu.angularVelocity)*0.5)*(q0 + 0.5*delta_t*k1);
	Eigen::Vector4d k3 = 0.5 * Rotation::bigOmegaMatrix((imu_l1.angularVelocity + imu.angularVelocity)*0.5)*(q0 + 0.5*delta_t*k2);
	Eigen::Vector4d k4 = 0.5 * Rotation::bigOmegaMatrix(imu_l1.angularVelocity)*(q0 + delta_t*k3);
	Eigen::Vector4d q_Bl1_Bl_JPL = q0 + (k1 + 2*k2 + 2*k3 + k4) * delta_t / 6.0;
	Eigen::Quaterniond q_Bl1_Bl(q_Bl1_Bl_JPL(3), q_Bl1_Bl_JPL(0), q_Bl1_Bl_JPL(1), q_Bl1_Bl_JPL(2));
	q_Bl1_Bl.normalize();
	assert(!std::isnan(q_Bl1_Bl.norm()));

	//accelerometer measurements
	const Eigen::Vector3d global_gravity = parameter_->getGlobalGravity();
	Eigen::Matrix3d R_Bl_Bl1 = q_Bl1_Bl.conjugate().toRotationMatrix();
	Eigen::Vector3d s_hat = (R_Bl_Bl1*imu_l1.acceleration + imu.acceleration) * delta_t * 0.5;
	Eigen::Vector3d y_hat = s_hat * delta_t * 0.5;
	Eigen::Matrix3d R_G_Bl = q_B_G.conjugate().toRotationMatrix();
	Eigen::Vector3d v_hat_l1 = v_B_G + R_G_Bl * s_hat - global_gravity * delta_t;
	Eigen::Vector3d p_hat_l1 = p_B_G + v_B_G * delta_t + R_G_Bl * y_hat - 0.5*delta_t*delta_t*global_gravity;

	//update state_l1
	state_l1.imu = imu_l1;
	state_l1.s = s_hat;
	state_l1.y = y_hat;
	state_l1.q_B_G = q_Bl1_Bl * q_B_G;
	state_l1.v_B_G = v_hat_l1;
	state_l1.p_B_G = p_hat_l1;
}

Eigen::Matrix<double, 15, 15> BodyState::getBodyStateTransitionMatrix(BodyState &state_l1) {
	Eigen::Matrix<double, 15, 15> bodyStateTransitionMatrix = Eigen::Matrix<double, 15, 15>::Identity();
	double deltaT = state_l1.imu.time - imu.time;

	Eigen::Matrix3d R_Bl_G_T = q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d R_Bl1_G_T = state_l1.q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d pi_p_q = -Rotation::crossMatrix(R_Bl_G_T*state_l1.y);
	Eigen::Matrix3d pi_v_q = -Rotation::crossMatrix(R_Bl_G_T*state_l1.s);
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

	Eigen::Vector3d global_gravity = parameter_->getGlobalGravity();

	return bodyStateTransitionMatrix;
}

Eigen::VectorXd BodyState::getBodyStateVector() {
	bodyStateVector_(0) = q_B_G.w();
	bodyStateVector_(1) = q_B_G.x();
	bodyStateVector_(2) = q_B_G.y();
	bodyStateVector_(3) = q_B_G.z();
	bodyStateVector_.segment<3>(4) = p_B_G;
	bodyStateVector_.segment<3>(7) = v_B_G;
	bodyStateVector_.segment<3>(10) = b_g;
	bodyStateVector_.segment<3>(13) = b_a;
	return bodyStateVector_;
}
