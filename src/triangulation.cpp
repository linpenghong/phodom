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
//for test
#include "camera_reprojection_functor.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>
#include <eigen3/unsupported/Eigen/LevenbergMarquardt>

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

//        std::cout << "bodyPose = " <<  it->getBodyPositionInGlobalFrame().transpose() << std::endl;
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
    Eigen::VectorXd xx = x;
    std::cout << "initial_guess = " << initial_guess.transpose() << std::endl;
    // Solve by LM method
    CameraReprojectionFunctor functor(rotations, positions, measurements, *filter_);
    Eigen::NumericalDiff<CameraReprojectionFunctor> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraReprojectionFunctor>> lm(num_diff);
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);

    std::cout << "standard result = " << getRealParam(x).transpose() << std::endl;

//    std::cout << "first R  =\n" << rotations[0] << std::endl;
//    std::cout << "feature0 = " << feature_track[0].array() << std::endl;
//    std::cout << "featuren = " << feature_track[n-1].array() << std::endl;
//    std::cout << "z0       = " << z0.transpose() << std::endl;
//    std::cout << "z_last   = " << z_last.transpose() << std::endl;


    Eigen::MatrixXd  f_Extend(2*n, 1);
    Eigen::MatrixXd Jf_Extend(2*n, 3);

    for(std::size_t ii=0; ii < 20; ++ii) {
    	for(std::size_t jj = 0; jj < n; ++jj) {
    		 f_Extend.block<2,1>(2*jj, 0) = measurements[jj] - cameraProject(rotations[jj]*getRealParam(xx)+positions[jj]);
    		Jf_Extend.block<2,3>(2*jj, 0) = cameraProjectJacobian(rotations[jj], positions[jj], getRealParam(xx));
    	}

//    	std::cout << "Jf_Extend = " << Jf_Extend << std::endl;
//    	std::cout << " f_Extend = " <<  f_Extend << std::endl;

    	xx -= (Jf_Extend.transpose()*Jf_Extend).inverse()*(Jf_Extend.transpose()*f_Extend);
//    	std::cout << "x = " << x.transpose() << std::endl;
    }
    std::cout << "positon in camera frame = " << getRealParam(xx).transpose() << std::endl;

    Eigen::Vector3d global_position;

    global_position = it_c0->getCameraOrientationInGlobalFrame(*filter_).conjugate().toRotationMatrix()*getRealParam(x) + it_c0->getCameraPositionInGlobalFrame(*filter_);
	return std::make_pair(true, global_position);
}

Eigen::Vector3d Triangulation::getRealParam(const Eigen::Vector3d& fakeParam) {
	Eigen::Vector3d realParam;
	realParam(2) = 1.0/fakeParam(2);
	realParam(0) = realParam(2) * fakeParam(0);
	realParam(1) = realParam(2) * fakeParam(1);
	return realParam;
}

Eigen::Vector3d Triangulation::getFakeParam(const Eigen::Vector3d& realParam) {
	Eigen::Vector3d fakeParam;
	fakeParam(0) = realParam(0)/realParam(2);
	fakeParam(0) = realParam(1)/realParam(2);
	fakeParam(0) = 1.0/realParam(2);
	return fakeParam;
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

//TODO::这里计算效率有待改进，中间量可以利用
Eigen::Matrix<double, 2, 3> Triangulation::cameraProjectJacobian(const Eigen::Matrix3d& R_c, const Eigen::Vector3d& p_c, const Eigen::Vector3d& p) const {
	Eigen::Matrix<double, 2, 3> J_f;
    double x = p(0);
    double y = p(1);
    double z = p(2);
    double u = x / z;
    double v = y / z;
    double r = u*u + v*v;
    double k1 = filter_->radial_distortion_(0);
    double k2 = filter_->radial_distortion_(1);
    double k3 = filter_->radial_distortion_(2);
    double t1 = filter_->tangential_distortion_(0);
    double t2 = filter_->tangential_distortion_(1);
    double dr = 1.0 + k1*r + k2*r*r + k3*std::pow(r, 3.0);
    double x2_y2 = x*x + y*y;

    Eigen::RowVector3d uv_by_xyz;
    Eigen::RowVector3d u_by_xyz;
    Eigen::RowVector3d v_by_xyz;
    Eigen::RowVector3d r_by_xyz;
    Eigen::RowVector3d rr_by_xyz;
    Eigen::RowVector3d rrr_by_xyz;
    Eigen::RowVector3d dr_by_xyz;
    Eigen::Matrix<double, 2, 3> dt_by_xyz;

    uv_by_xyz << y/(z*z), x/(z*z), -2.0*x*y/std::pow(z, 3.0);
    u_by_xyz << 1.0/z, 0.0, -1.0*x/(z*z);
    v_by_xyz << 0.0, 1.0/z, -1.0*y/(z*z);
    r_by_xyz << 2.0*x/(z*z), 2.0*y/(z*z), -2.0*(x2_y2)/std::pow(z, 3.0);
    rr_by_xyz << 2.0*r*r_by_xyz(0), 2.0*r*r_by_xyz(1), 2.0*r*r_by_xyz(2);
    rrr_by_xyz << 3.0*r*r*r_by_xyz(0), 3.0*r*r*r_by_xyz(1), 3.0*r*r*r_by_xyz(2);
    dr_by_xyz << k1*r_by_xyz + k2*rr_by_xyz + k3*rrr_by_xyz;
    dt_by_xyz.block<1, 3>(0, 0) = 2.0*t1*uv_by_xyz + t2*r_by_xyz + 4.0*t2*u*u_by_xyz;
    dt_by_xyz.block<1, 3>(1, 0) = 2.0*t2*uv_by_xyz + t1*r_by_xyz + 4.0*t1*v*v_by_xyz;

    Eigen::Matrix2d h_by_g_L = filter_->focal_point_.asDiagonal();
    Eigen::Matrix<double, 2, 3> h_by_g_R;
    Eigen::Matrix<double, 2, 3> h_by_g;
    h_by_g_R(0, 0) = dr/z + dr_by_xyz(0)*u + dt_by_xyz(0);
    h_by_g_R(0, 1) = dr_by_xyz(1)*u + dt_by_xyz(1);
    h_by_g_R(0, 2) = -1.0*dr*u/z + dr_by_xyz(2)*u + dt_by_xyz(2);
    h_by_g_R(1, 0) = dr_by_xyz(0)*v + dt_by_xyz(0);
    h_by_g_R(1, 1) = dr/z + dr_by_xyz(1)*v + dt_by_xyz(1);
    h_by_g_R(1, 2) = -1.0*dr*v/z + dr_by_xyz(2)*v + dt_by_xyz(2);
    h_by_g = h_by_g_L*h_by_g_R;

    Eigen::Matrix3d g_by_theta = R_c;
    g_by_theta.block<3,1>(0,2) = p_c;

    J_f = h_by_g * g_by_theta;
    return J_f;
}

