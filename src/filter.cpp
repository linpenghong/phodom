/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:filter.cpp
*   Author  :linpenghong
*   Date    :Jan 18, 2017
*   Describe:TODO
*
********************************************************/
#include "filter.h"
#include "parameter.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <iostream>

Filter::Filter(std::shared_ptr<Parameter> parameter_, std::shared_ptr<ImuBuffer> imuBuffer_) : bodyState(parameter_) {
	parameter = parameter_;
	imuBuffer = imuBuffer_;
	propaState.setZero(57);
	augmentState.setZero(57);
	propaCovariance.setZero(56,56);
	augmentCovariance.setZero(56,56);
}

void Filter::computeImuEstimate(boost::circular_buffer<ImuItem>::iterator it) {
	imuEstimate_.time = it->time;
	imuEstimate_.acceleration = imuState.T_a.inverse() * (it->acceleration - bodyState.b_a);
	imuEstimate_.angularVelocity = imuState.T_g.inverse() * (it->angularVelocity - imuState.T_s*imuEstimate_.acceleration - bodyState.b_g);
	assert(imuEstimate_.acceleration.norm() < 1000);
	assert(imuEstimate_.angularVelocity.norm() < 1000);
}

void Filter::propagateToTime(double time) {
	assert(!imuBuffer->data.empty());
//	assert(imuBuffer->data.begin()->time < time);

//	imuBuffer->print();

	while(!imuBuffer->data.empty() && imuBuffer->data.begin()->time <= time) {
		computeImuEstimate(imuBuffer->data.begin());
		BodyState nextState(bodyState);
		bodyState.propagateBodyState(nextState, imuEstimate_);
		bodyState.propagateCovariance(*this, nextState);
		bodyState = nextState;
		imuBuffer->data.pop_front();
	}
	std::cout << "New Body State Q: " << bodyState.q_B_G.x() << ", " << bodyState.q_B_G.y() << ", " << bodyState.q_B_G.z() << ", " << bodyState.q_B_G.w() << std::endl;
	std::cout << "New Body State P: " << bodyState.p_B_G.x() << ", " << bodyState.p_B_G.y() << ", " << bodyState.p_B_G.z() << std::endl;
	std::cout << "New Body State V: " << bodyState.v_B_G.x() << ", " << bodyState.v_B_G.y() << ", " << bodyState.v_B_G.z() << std::endl;
	std::cout << "New Covariance :\n" << bodyState.covariance.block<15, 15>(0, 0) << std::endl;
}

void Filter::stepImage(double time, cv::Mat& frame, const ImuBuffer::iterator& hint_gyro, const ImuBuffer::iterator& hint_accel) {
    if (!bodyState.is_initialized) {
        return;
    }
//    stateAugment();
//    CameraPose& last_camera_pose = state().poses().back();
//
//    frame_rows_ = frame.rows;
//    Track::feature_track_list current_features_tracked;
//    if (feature_tracker_.previous_frame_features_.keypoints().size() == 0) {
//        assert(state().poses().size() == 1);
//    }
//    current_features_tracked = feature_tracker_.processImage(features_tracked_, frame);
//    last_camera_pose.setActiveFeaturesCount(current_features_tracked.size());
//    for (std::size_t i = 0; i < current_features_tracked.size(); ++i) {
//        if (current_features_tracked[i]->wasUsedForResidualization()) {
//            last_camera_pose.rememberFeatureId(current_features_tracked[i]->feature_id);
//            last_camera_pose.decreaseActiveFeaturesCount(current_features_tracked[i]->feature_id);
//        } else {
//            last_camera_pose.rememberFeatureId(current_features_tracked[i]->feature_id);
//        }
//    }
//
//    Track::feature_track_list features_to_rezidualize;
//    for (std::size_t i = 0; i < features_tracked_.size(); ++i) {
//        assert(features_tracked_[i]->positions.size() <= parameter->getMaxCameraPoses() + 1 || features_tracked_[i]->wasUsedForResidualization());
//        if (features_tracked_[i]->wasUsedForResidualization()) {
//            continue;
//        }
//        if (features_tracked_[i]->isOutOfView()) {
//            features_tracked_[i]->setUsedForResidualization();
//            features_to_rezidualize.push_back(features_tracked_[i]);
//        } else if (features_tracked_[i]->posesTrackedCount() == parameter->getMaxCameraPoses() + 1) {
//            features_tracked_[i]->setUsedForResidualization();
//            features_tracked_[i]->popFeaturePosition();
//            last_camera_pose.decreaseActiveFeaturesCount(features_tracked_[i]->getFeatureId());
//            features_to_rezidualize.push_back(features_tracked_[i]);
//        }
//    }
}
