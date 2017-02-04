/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:camera_pose.h
*   Author  :linpenghong
*   Date    :Jan 11, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_CAMERA_POSE_H_
#define PHODOM_INCLUDE_CAMERA_POSE_H_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <set>
#include <memory>

#include "imu_buffer.h"

class BodyState;
class Filter;

class CameraPose {
	friend class Filter;
public:
    CameraPose(const BodyState& body_state, ImuBuffer::iterator hintImu);

    std::size_t getActiveFeaturesCount() const;
    void setActiveFeaturesCount(std::size_t i);
    void decreaseActiveFeaturesCount(int feature_id);

    BodyState& getBodyState();
    const BodyState& getBodyState() const;

    double time() const;
    const Eigen::Quaterniond& getBodyOrientationInGlobalFrame() const;

    Eigen::Quaterniond getCameraOrientationInGlobalFrame(const Filter& filter) const;

    const Eigen::Vector3d& getBodyPositionInGlobalFrame() const;

    Eigen::Vector3d getCameraPositionInGlobalFrame(const Filter& filter) const;

    const Eigen::Vector3d& getBodyVelocityInGlobalFrame() const;

    Eigen::Quaterniond getRotationToOtherPose(const CameraPose& other, const Filter& filter) const;

    Eigen::Vector3d getPositionOfAnotherPose(const CameraPose& other, const Filter& filter) const;

    void rememberFeatureId(int feature_id);

    void updateWithStateDelta(const Eigen::VectorXd& delta_x);

    ImuBuffer::iterator imuHint() const;

    std::size_t getCameraPoseId() const;

private:
    static std::size_t camera_pose_counter;
    std::size_t camera_pose_id_;
    std::set<int> feature_ids_;
    std::size_t features_active_;
    std::shared_ptr<BodyState> body_state_;
    ImuBuffer::iterator hint_imu_;
};


#endif /* PHODOM_INCLUDE_CAMERA_POSE_H_ */
