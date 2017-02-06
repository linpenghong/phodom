/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:triangulation.cpp
*   Author  :linpenghong
*   Date    :Feb 4, 2017
*   Describe:TODO
*
********************************************************/

#include "triangulation.h"

#include <string>
#include <iostream>

#include "feature_track.h"
#include "filter.h"

Triangulation::Triangulation(const Filter* filter) : filter_(filter) {

}

Eigen::Vector3d Triangulation::initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1,
		const Eigen::Matrix3d& R_C0_C1, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const {

    Eigen::Vector3d v1;
    v1 << z0(0), z0(1), 1.0;
    Eigen::Vector3d v2;
    v2 << z1(0), z1(1), 1.0;

    v1.normalize();
    v2.normalize();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a(3, 2);
    a.block<3, 1>(0, 0) = v1;
    a.block<3, 1>(0, 1) = -1*R_C0_C1*v2;
    Eigen::Vector3d b = p_C1_C0;

    Eigen::Vector2d x;

    if (method == InitialGuessMethod::SVD) {
        x = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    } else if (method == InitialGuessMethod::QR) {
        x = a.colPivHouseholderQr().solve(b);
    } else if (method == InitialGuessMethod::normal) {
        x = (a.transpose() * a).ldlt().solve(a.transpose() * b);
    } else {
        throw std::invalid_argument("ReprojectionOptimizer::initialGuess method must be either SVD, QR or normal");
    }

    return x(0) * v1;
}

std::pair<bool, Eigen::Vector3d> Triangulation::getFeaturePos(const FeatureTrack &feature_track){
    std::size_t n = feature_track.positions.size();

    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector2d> measurements;

    const CameraPoseBuffer& poses = filter_->cameraPoseBuf;
    assert(n <= poses.size());
    CameraPoseBuffer::const_iterator it_c0 = std::next(poses.begin(), poses.size() - (n+1));
    CameraPoseBuffer::const_iterator it_end = std::prev(std::end(poses));
    for (auto it = it_c0; it != it_end; ++it) {

        std::cout << "bodyPose = " <<  it->getBodyPositionInGlobalFrame().transpose() << std::endl;
        const CameraPose& c0 = *it_c0;
        const CameraPose& ci = *it;

        Eigen::Quaterniond q_Ci_C0 = c0.getRotationToOtherPose(ci, *filter_);
        Eigen::Vector3d p_C0_Ci = ci.getPositionOfAnotherPose(c0, *filter_);
        Eigen::Vector2d z_i = feature_track[it - it_c0];

        rotations.push_back(q_Ci_C0.toRotationMatrix());
        positions.push_back(p_C0_Ci);
        measurements.push_back(z_i);
    }

    // Initial guess
    const Eigen::Vector2d& z0 = (feature_track[0].array() - filter_->optical_center_.array()) / filter_->focal_point_.array();
    const Eigen::Vector2d& z_last = (feature_track[n-1].array() - filter_->optical_center_.array()) / filter_->focal_point_.array();
    const Eigen::Quaterniond q_Clast_C0 = (it_c0)->getRotationToOtherPose(*(it_c0 + (n-1)), *filter_);
    const Eigen::Vector3d& p_Clast_C0 = (it_c0)->getPositionOfAnotherPose(*(it_c0 + (n-1)), *filter_);
    Eigen::Vector3d initial_guess = initialGuessFeaturePosition(z0, z_last, q_Clast_C0.conjugate().toRotationMatrix(), p_Clast_C0, InitialGuessMethod::SVD);
    Eigen::VectorXd x = initial_guess;
    x(2) = 1.0;
    x /= initial_guess(2);


    std::cout << "feature0 = " << feature_track[0].array() << std::endl;
    std::cout << "featuren = " << feature_track[n-1].array() << std::endl;
    std::cout << "z0       = " << z0.transpose() << std::endl;
    std::cout << "z_last   = " << z_last.transpose() << std::endl;
    std::cout << "initial_guess = " << initial_guess << std::endl;

    {
        Eigen::VectorXd xs(measurements.size());
        Eigen::VectorXd ys(measurements.size());
        for (std::size_t i = 0; i < measurements.size(); ++i) {
            xs(i) = measurements[i](0);
            ys(i) = measurements[i](1);
        }
        Eigen::Matrix3d R_Clast_C0 = q_Clast_C0.toRotationMatrix();
        Eigen::IOFormat formatter(4, 0, ", ", "\n", "[", "]");
        Eigen::Matrix<double, 9, 1> r;
        r.block<3, 1>(0, 0) = R_Clast_C0.block<1, 3>(0, 0);
        r.block<3, 1>(3, 0) = R_Clast_C0.block<1, 3>(1, 0);
        r.block<3, 1>(6, 0) = R_Clast_C0.block<1, 3>(2, 0);
    }


    Eigen::Vector3d global_position;

    global_position.setZero();

	return std::make_pair(true, global_position);
}

Eigen::Vector3d Triangulation::gFunction(Eigen::Matrix3d R_Ci_C0, Eigen::Vector3d p_Ci_C0, Eigen::Vector3d param) {
	Eigen::Vector3d temp;
	temp << param(0), param(1), 1;
	return R_Ci_C0*temp + param(3)*p_Ci_C0;
}

Eigen::Vector2d Triangulation::cameraProject(const Eigen::Vector3d& p) const {
    double u = p(0) / p(2);
    double v = p(1) / p(2);
    Eigen::Vector2d uv_vec;
    uv_vec << u, v;
    double r = u*u + v*v;
    double k_1 = filter_->radial_distortion_[0];
    double k_2 = filter_->radial_distortion_[1];
    double k_3 = filter_->radial_distortion_[2];
    double t_1 = filter_->tangential_distortion_[0];
    double t_2 = filter_->tangential_distortion_[1];
    double d_r = 1 + k_1*r + k_2*r*r + k_3*r*r*r;
    Eigen::Vector2d d_t;
    d_t << 2*u*v*t_1 + (r + 2*u*u)*t_2,
    		2*u*v*t_2 + (r + 2*v*v)*t_1;
    return filter_->optical_center_ + filter_->focal_point_.asDiagonal()*(d_r*uv_vec + d_t);
}


