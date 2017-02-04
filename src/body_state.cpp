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
#include "filter.h"
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

//TODO: add noise covariance
void BodyState::propagateCovariance(const Filter& filter, BodyState &state_l1) {
	Eigen::Matrix<double, 15, 15> bodyStateTransitionM = BodyState::getBodyStateTransitionMatrix(filter, state_l1);
	Eigen::Matrix<double, 56, 56> transitionM = Eigen::Matrix<double, 56, 56>::Identity();
	transitionM.block<15, 15>(0, 0) = bodyStateTransitionM;
	transitionM.block<15, 27>(0, 15) = BodyState::getImuCalibrationParamsTransitionMatrix(filter, state_l1);

	state_l1.covariance = transitionM * covariance * transitionM.transpose();

//	std::cout << "New Transition :\n" << transitionM.block<15, 15>(0, 0) << std::endl;

	state_l1.covariance.block<15, 15>(0, 0) += BodyState::propagationNoiseMatrix(filter, state_l1, bodyStateTransitionM);
}

//TODO: 这里的一些中间数据可以在算Gamma的时候重复利用，提升效率
Eigen::Matrix<double, 15, 15> BodyState::getBodyStateTransitionMatrix(const Filter& filter, BodyState &state_l1) {
	Eigen::Matrix<double, 15, 15> bodyStateTransitionMatrix = Eigen::Matrix<double, 15, 15>::Identity();
	double deltaT = state_l1.imu.time - imu.time;
	double half_deltaT = 0.5 * deltaT;
	double squared_half_deltaT = half_deltaT*half_deltaT;
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
	Eigen::Vector3d global_gravity = parameter_->getGlobalGravity();

	Eigen::Matrix3d R_Bl_G_T = q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d R_Bl1_G_T = state_l1.q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d R_T_sum = R_Bl_G_T + R_Bl1_G_T;
	Eigen::Matrix3d T_g_inv = filter.imuState.T_g.inverse();
	Eigen::Matrix3d T_a_inv = filter.imuState.T_a.inverse();
	Eigen::Vector3d a_G_l1	= R_Bl1_G_T * state_l1.imu.acceleration - global_gravity;
	Eigen::Matrix3d cross_a_G_l1 = Rotation::crossMatrix(a_G_l1);

	Eigen::Matrix3d phi_p_q = -Rotation::crossMatrix(R_Bl_G_T*state_l1.y);
	Eigen::Matrix3d phi_v_q = -Rotation::crossMatrix(R_Bl_G_T*state_l1.s);
	Eigen::Matrix3d phi_q_bg = -half_deltaT * R_T_sum * T_g_inv;
	Eigen::Matrix3d phi_v_bg = squared_half_deltaT * (cross_a_G_l1 * R_T_sum) * T_g_inv;//different from B.37
	Eigen::Matrix3d phi_p_bg = half_deltaT * phi_v_bg;
	Eigen::Matrix3d phi_q_ba = half_deltaT * R_T_sum * T_g_inv * filter.imuState.T_s * T_a_inv;
	Eigen::Matrix3d phi_v_ba = -half_deltaT * R_T_sum * T_a_inv + squared_half_deltaT * cross_a_G_l1 * R_T_sum * T_g_inv * filter.imuState.T_s * T_a_inv;
	Eigen::Matrix3d phi_p_ba = half_deltaT * phi_v_ba;

	bodyStateTransitionMatrix.block<3, 3>(3, 0) = phi_p_q;
	bodyStateTransitionMatrix.block<3, 3>(6, 0) = phi_v_q;
	bodyStateTransitionMatrix.block<3, 3>(3, 6) = deltaT * identity;
	bodyStateTransitionMatrix.block<3, 3>(0, 9) = phi_q_bg;
	bodyStateTransitionMatrix.block<3, 3>(0, 12) = phi_q_ba;
	bodyStateTransitionMatrix.block<3, 3>(3, 9) = phi_p_bg;
	bodyStateTransitionMatrix.block<3, 3>(3, 12) = phi_p_ba;
	bodyStateTransitionMatrix.block<3, 3>(6, 9) = phi_v_bg;
	bodyStateTransitionMatrix.block<3, 3>(6, 12) = phi_v_ba;

	return bodyStateTransitionMatrix;
}

Eigen::Matrix<double, 15, 27> BodyState::getImuCalibrationParamsTransitionMatrix(const Filter& filter, BodyState &state_l1) {

	Eigen::Matrix<double, 15, 27> gamma_imu = Eigen::Matrix<double, 15, 27>::Zero();

//	return gamma_imu;

	double deltaT = state_l1.imu.time - imu.time;
	double half_deltaT = 0.5 * deltaT;
	double squared_half_deltaT = half_deltaT*half_deltaT;
	Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
	Eigen::Vector3d global_gravity = parameter_->getGlobalGravity();

	Eigen::Matrix3d R_Bl_G_T = q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d R_Bl1_G_T = state_l1.q_B_G.conjugate().toRotationMatrix();
	Eigen::Matrix3d R_T_sum = R_Bl_G_T + R_Bl1_G_T;
	Eigen::Matrix3d T_g_inv = filter.imuState.T_g.inverse();
	Eigen::Matrix3d T_a_inv = filter.imuState.T_a.inverse();
	Eigen::Vector3d a_G_l1	= R_Bl1_G_T * state_l1.imu.acceleration - global_gravity;
	Eigen::Matrix3d cross_a_G_l1 = Rotation::crossMatrix(a_G_l1);
	Eigen::Matrix<double, 3, 9> derivate_Tg_w = Rotation::derivativeMatrixByVector(state_l1.imu.angularVelocity);
	Eigen::Matrix<double, 3, 9> derivate_Ts_a = Rotation::derivativeMatrixByVector(state_l1.imu.acceleration);
	Eigen::Matrix<double, 3, 9> derivate_Ta_a = Rotation::derivativeMatrixByVector(state_l1.imu.acceleration);

//	Eigen::Matrix<double, 3, 9> zero_block = Eigen::Matrix<double, 3, 9>::Zero();
	Eigen::Matrix<double, 3, 9> gamma_q_Tg = -half_deltaT * R_T_sum * T_g_inv * derivate_Tg_w;
	Eigen::Matrix<double, 3, 9> gamma_v_Tg = -squared_half_deltaT * cross_a_G_l1 * R_T_sum * T_g_inv * derivate_Tg_w;
	Eigen::Matrix<double, 3, 9> gamma_p_Tg = half_deltaT * gamma_v_Tg;
	Eigen::Matrix<double, 3, 9> gamma_q_Ts = -half_deltaT * R_T_sum * T_g_inv * derivate_Ts_a;
	Eigen::Matrix<double, 3, 9> gamma_v_Ts = -squared_half_deltaT * cross_a_G_l1 * R_T_sum * T_g_inv * derivate_Ts_a;
	Eigen::Matrix<double, 3, 9> gamma_p_Ts = half_deltaT * gamma_v_Ts;
	Eigen::Matrix<double, 3, 9> gamma_q_Ta = half_deltaT * R_T_sum * T_g_inv * filter.imuState.T_s * T_a_inv * derivate_Ta_a;
	Eigen::Matrix<double, 3, 9> gamma_v_Ta = squared_half_deltaT * cross_a_G_l1 * R_T_sum * T_g_inv * filter.imuState.T_s * T_a_inv * derivate_Ta_a;
	Eigen::Matrix<double, 3, 9> gamma_p_Ta = half_deltaT * gamma_v_Ta;

	gamma_imu.block<3, 9>(0, 0) = gamma_q_Tg;
	gamma_imu.block<3, 9>(0, 9) = gamma_q_Ts;
	gamma_imu.block<3, 9>(0, 18) = gamma_q_Ta;
	gamma_imu.block<3, 9>(3, 0) = gamma_p_Tg;
	gamma_imu.block<3, 9>(3, 9) = gamma_p_Ts;
	gamma_imu.block<3, 9>(3, 18) = gamma_p_Ta;
	gamma_imu.block<3, 9>(6, 0) = gamma_v_Tg;
	gamma_imu.block<3, 9>(6, 9) = gamma_v_Ts;
	gamma_imu.block<3, 9>(6, 18) = gamma_v_Ta;

	return gamma_imu;
}

Eigen::Matrix<double, 15, 15> BodyState::propagationNoiseMatrix(const Filter& filter, BodyState& state_l1,
			const Eigen::Matrix<double, 15, 15>& bodyStateTransitionMatrix) {
	double deltaT = state_l1.imu.time - imu.time;

    double gyroscope_variance = filter.parameter->getGyroscopeVariance();
    double accelerometer_variance = filter.parameter->getAccelerometerVariance();
    double gyroscope_random_walk_variance = filter.parameter->getGyroscopeRandomWalkVariance();
    double accelerometer_random_walk_variance = filter.parameter->getAccelerometerRandomWalkVariance();

    double simga_gc_squared = gyroscope_variance*gyroscope_variance;
    double sigma_ga_squared = accelerometer_variance*accelerometer_variance;
    double sigma_wgc_squared = gyroscope_random_walk_variance*gyroscope_random_walk_variance;
    double sigma_wga_squared = accelerometer_random_walk_variance*accelerometer_random_walk_variance;

//    double delta_t = from_state.timeTo(to_state);
    Eigen::Matrix3d R_Bl_G_T = q_B_G.conjugate().toRotationMatrix();

    Eigen::Matrix<double, 12, 12> q_c = Eigen::Matrix<double, 12, 12>::Zero();
    q_c.block<3, 3>(0, 0) = simga_gc_squared*Eigen::Matrix3d::Identity();
    q_c.block<3, 3>(3, 3) = sigma_ga_squared*Eigen::Matrix3d::Identity();
    q_c.block<3, 3>(6, 6) = sigma_wgc_squared*Eigen::Matrix3d::Identity();
    q_c.block<3, 3>(9, 9) = sigma_wga_squared*Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 12> g_c = Eigen::Matrix<double, 15, 12>::Zero();
    g_c.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    g_c.block<3, 3>(6, 3) = -R_Bl_G_T;
    g_c.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
    g_c.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 15> n_c = g_c*q_c*g_c.transpose();
    Eigen::Matrix<double, 15, 15> q_d = 0.5*deltaT*bodyStateTransitionMatrix*n_c*bodyStateTransitionMatrix.transpose() + n_c;

//    std::cout << "New NoiseCov :\n" << q_d << std::endl;
//    q_d.setZero();

    return q_d;
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

void BodyState::updateWithStateDelta(const Eigen::VectorXd& delta_x) {
    Eigen::Quaterniond delta_q(0.5*delta_x(0), 0.5*delta_x(1), 0.5*delta_x(2), 1.0);
    delta_q.normalize();
    q_B_G = delta_q*q_B_G;
    q_B_G.normalize();
    p_B_G += delta_x.segment<3>(3);
    v_B_G += delta_x.segment<3>(6);
}
