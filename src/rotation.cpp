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
