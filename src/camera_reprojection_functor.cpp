/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:camera_reprojection_functor.cpp
*   Author  :linpenghong
*   Date    :Jan 12, 2017
*   Describe:TODO
*
********************************************************/
#include "camera_reprojection_functor.h"

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/LevenbergMarquardt>
#include <vector>

#include "filter.h"

CameraReprojectionFunctor::CameraReprojectionFunctor(
        const std::vector<Eigen::Matrix3d>& rotations, const std::vector<Eigen::Vector3d>& positions,
        const std::vector<Eigen::Vector2d>& measurements, const Filter& filter)
    : Eigen::DenseFunctor<double>(), rotations_(rotations), positions_(positions), measurements_(measurements),
    filter_(filter) {}

int CameraReprojectionFunctor::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
{
    std::size_t n = rotations_.size();
    assert(positions_.size() == n);
    assert(measurements_.size() == n);
    assert(x.rows() == inputs());
    assert(fvec.rows() == values());

    for (std::size_t i = 0; i < n; ++i) {
        fvec.segment<2>(2*i) = measurements_[i] - filter_.triangulation.cameraProject(g(i, x));
    }

    return 0;
}

int CameraReprojectionFunctor::inputs() const {
    return 3;
}

int CameraReprojectionFunctor::values() const {
    return 2 * rotations_.size();
}

Eigen::Vector3d CameraReprojectionFunctor::g(int i, const Eigen::Vector3d& est) const {
    Eigen::Vector3d params;
    params << est(0), est(1), 1.0;
    double rho = est(2);
    return rotations_[i]*params + rho*positions_[i];
}
