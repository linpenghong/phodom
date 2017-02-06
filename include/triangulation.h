/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:triangulation.h
*   Author  :linpenghong
*   Date    :Feb 4, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_TRIANGULATION_H_
#define PHODOM3_INCLUDE_TRIANGULATION_H_

#include <eigen3/Eigen/Core>
#include <math.h>

class Filter;
class FeatureTrack;

enum InitialGuessMethod {
    SVD, QR, normal
};

class Triangulation {
//	friend class Filter;
public:

	Triangulation(const Filter* filter);
    Eigen::Vector3d initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1,
            const Eigen::Matrix3d& R_C1_C0, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const;
	std::pair<bool, Eigen::Vector3d> getFeaturePos(const FeatureTrack &feature_track);
	Eigen::Vector3d gFunction(Eigen::Matrix3d R_Ci_C0, Eigen::Vector3d p_Ci_C0, Eigen::Vector3d param);
	Eigen::Vector2d cameraProject(const Eigen::Vector3d& p) const;
private:
    const Filter* filter_;
};





#endif /* PHODOM3_INCLUDE_TRIANGULATION_H_ */
