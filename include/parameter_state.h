/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:parameter_state.h
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_PARAMETER_STATE_H_
#define PHODOM3_INCLUDE_PARAMETER_STATE_H_

#include <eigen3/Eigen/Core>
#include <iostream>

class CameraState {
public:
	CameraState();
	void setCameraStateVector(Eigen::VectorXd other);
	Eigen::VectorXd getCameraStateVector();
private:
	Eigen::VectorXd cameraStateVector_;
};

class ParameterState {
public:
	ParameterState();
	void setParameterStateVector(Eigen::VectorXd other);
	Eigen::VectorXd getParameterStateVector();

	Eigen::Vector3d p_B_C;
	CameraState x_cam;
	double t_d;
	double t_r;
private:
	Eigen::VectorXd parameterStateVector_;
};

#endif /* PHODOM3_INCLUDE_PARAMETER_STATE_H_ */
