/*
 * rotation.h
 *
 *  Created on: Jan 16, 2017
 *      Author: lph
 */

#ifndef PHODOM2_INCLUDE_ROTATION_H_
#define PHODOM2_INCLUDE_ROTATION_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Rotation {
public:
	static Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v);
	static Eigen::Matrix4d bigOmegaMatrix(const Eigen::Vector3d& v);
	static Eigen::Matrix<double, 3, 9> derivativeMatrixByVector(const Eigen::Vector3d& v);
	static Eigen::Quaterniond fromTwoVectors(const Eigen::Vector3d& vFrom, const Eigen::Vector3d& vTo);
};

#endif /* PHODOM2_INCLUDE_ROTATION_H_ */
