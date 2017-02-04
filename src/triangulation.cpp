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
#include "filter.h"
#include "feature_track.h"

#include <string>
#include <iostream>

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
    assert(n <= poses.size() - 1);
    CameraPoseBuffer::const_iterator it_c0 = std::next(poses.begin(), poses.size() - (n+1));
    CameraPoseBuffer::const_iterator it_end = std::prev(std::end(poses));
    for (auto it = it_c0; it != it_end; ++it) {
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
        std::ofstream out("~/dump/feature_" + std::to_string(feature_track.feature_id) + ".txt");
        out << "{" << std::endl;
        out << "\"c0_pose_id\": " << it_c0->getCameraPoseId() << "," << std::endl;
        out << "\"z0\": " << z0.transpose().format(formatter) << "," << std::endl;
        out << "\"z_last\": " << z_last.transpose().format(formatter) << "," << std::endl;
        out << "\"p_Clast_C0\": " << p_Clast_C0.transpose().format(formatter) << "," << std::endl;
        out << "\"R_Clast_C0\": " << r.transpose().format(formatter) << "," << std::endl;
        out << "\"xs\": " << xs.transpose().format(formatter) << "," << std::endl;
        out << "\"ys\": " << ys.transpose().format(formatter) << "," << std::endl;
        out << "\"initial_guess\": " << initial_guess.transpose().format(formatter) << std::endl;
        out << "}" << std::endl;
    }

    Eigen::Vector3d global_position;

	return std::make_pair(true, global_position);
}
