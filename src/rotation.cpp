/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:rotation.cpp
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/

#include "rotation.h"
#include <math.h>

Eigen::Matrix3d Rotation::crossMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0, -1*v(2), v(1),
         v(2), 0, -1*v(0),
         -1*v(1), v(0), 0;
    return m;
}

Eigen::Matrix4d Rotation::bigOmegaMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix4d m;
    m(3, 3) = 0.0;
    m.block<3, 3>(0, 0) = -1*Rotation::crossMatrix(v);
    m.block<3, 1>(0, 3) = v;
    m.block<1, 3>(3, 0) = -1*v.transpose();
    return m;
}

Eigen::Matrix<double, 3, 9> Rotation::derivativeMatrixByVector(const Eigen::Vector3d &v) {

	Eigen::Matrix<double, 3, 9> m = Eigen::Matrix<double, 3, 9>::Zero();
	m.block<1, 3>(0, 0) = v.transpose();
	m.block<1, 3>(1, 3) = v.transpose();
	m.block<1, 3>(2, 6) = v.transpose();
	return m;
}

Eigen::Quaterniond Rotation::fromTwoVectors(const Eigen::Vector3d& vFrom, const Eigen::Vector3d& vTo) {
	Eigen::Vector3d cross = vFrom.cross(vTo);
	cross.normalize();
	double alpha = acos(vFrom.dot(vTo)/(vFrom.norm()*vTo.norm()));
	Eigen::Quaterniond quat(cos(alpha/2), sin(alpha/2)*cross(0), sin(alpha/2)*cross(1), sin(alpha/2)*cross(2));
	quat.normalize();
	return quat;
}
