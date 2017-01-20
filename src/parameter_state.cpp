/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:parameter_state.cpp
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/
#include "parameter_state.h"

CameraState::CameraState() {
	cameraStateVector_.setZero(9);
}

void CameraState::setCameraStateVector(Eigen::VectorXd other) {
	cameraStateVector_ = other;
}

Eigen::VectorXd CameraState::getCameraStateVector() {
	return cameraStateVector_;
}

ParameterState::ParameterState() : t_d(0), t_r(0) {
	p_B_C.setZero();
	parameterStateVector_.setZero(14);
}

void ParameterState::setParameterStateVector(Eigen::VectorXd other) {
	parameterStateVector_ = other;
}

Eigen::VectorXd ParameterState::getParameterStateVector() {
	parameterStateVector_.segment<3>(0) = p_B_C;
	parameterStateVector_.segment<9>(3) = x_cam.getCameraStateVector();
	parameterStateVector_(12) = t_d;
	parameterStateVector_(13) = t_r;
	return parameterStateVector_;
}
