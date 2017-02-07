/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:camera_reprojection_functor.h
*   Author  :linpenghong
*   Date    :Jan 12, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_CAMERA_REPROJECTION_FUNCTOR_H_
#define PHODOM_INCLUDE_CAMERA_REPROJECTION_FUNCTOR_H_

#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/LevenbergMarquardt>
#include <vector>

class Filter;

class CameraReprojectionFunctor : public Eigen::DenseFunctor<double> {
public:
    template <class T>
    friend class Eigen::LevenbergMarquardt;

    CameraReprojectionFunctor(const std::vector<Eigen::Matrix3d>& rotations,
            const std::vector<Eigen::Vector3d>& positions, const std::vector<Eigen::Vector2d>& measurements,
            const Filter& filter);

    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

    int inputs() const;
    int values() const;
private:
    /**
     * @brief Rotation from \f$ C_{0} \f$ to \f$ C_{i} \f$
     *
     * At i-th position, there is \f$ {}_{C_0}^{C_i}\mathbf{R} \f$
     */
    const std::vector<Eigen::Matrix3d>& rotations_;

    /**
     * @brief Position of \f$ C_{i} \f$ in \f$ C_{0} \f$.
     *
     * At i-th position, there is \f$ {}^{C_i}\mathbf{p}_{C_0} \f$
     */
    const std::vector<Eigen::Vector3d>& positions_;

    const std::vector<Eigen::Vector2d>& measurements_;

    /**
     * @brief Compute \f$ \mathbf{g}_i(\theta) \f$.
     */
    Eigen::Vector3d g(int i, const Eigen::Vector3d& est) const;

    const Filter& filter_;
};

#endif /* PHODOM_INCLUDE_CAMERA_REPROJECTION_FUNCTOR_H_ */
