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
#include <set>

Filter::Filter(std::shared_ptr<Parameter> parameter_, std::shared_ptr<ImuBuffer> imuBuffer_) : bodyState(parameter_), feature_tracker(100), cameraPoseBuf(200) {
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
#ifdef IMU_DEBUG
	std::cout << "New Body State Q: " << bodyState.q_B_G.x() << ", " << bodyState.q_B_G.y() << ", " << bodyState.q_B_G.z() << ", " << bodyState.q_B_G.w() << std::endl;
	std::cout << "New Body State P: " << bodyState.p_B_G.x() << ", " << bodyState.p_B_G.y() << ", " << bodyState.p_B_G.z() << std::endl;
	std::cout << "New Body State V: " << bodyState.v_B_G.x() << ", " << bodyState.v_B_G.y() << ", " << bodyState.v_B_G.z() << std::endl;
	std::cout << "New Covariance :\n" << bodyState.covariance.block<15, 15>(0, 0) << std::endl;
#endif
}

void Filter::stepImage(double time, cv::Mat& frame, const ImuBuffer::iterator& hint_imu) {
    if (!bodyState.is_initialized) {
        return;
    }

    //===================================TO Rewrite========================================//
//    stateAugment();

    CameraPose new_pose(bodyState, hint_imu);
    cameraPoseBuf.addNewCameraPose(std::move(new_pose));

    CameraPose& last_camera_pose = cameraPoseBuf.back();

    Track::feature_track_list current_features_tracked;
    if (feature_tracker.previous_frame_features_.keypoints().size() == 0) {
        assert(cameraPoseBuf.size() == 1);
    }
    current_features_tracked = feature_tracker.processImage(features_tracked, frame);
    last_camera_pose.setActiveFeaturesCount(current_features_tracked.size());
    for (std::size_t i = 0; i < current_features_tracked.size(); ++i) {
        if (current_features_tracked[i]->was_used_for_residualization) {
            last_camera_pose.rememberFeatureId(current_features_tracked[i]->feature_id);
            last_camera_pose.decreaseActiveFeaturesCount(current_features_tracked[i]->feature_id);
        } else {
            last_camera_pose.rememberFeatureId(current_features_tracked[i]->feature_id);
        }
    }

    Track::feature_track_list features_to_rezidualize;
    for (std::size_t i = 0; i < features_tracked.size(); ++i) {
        assert(features_tracked[i]->positions.size() <= parameter->getMaxCameraPoses() + 1 || features_tracked[i]->was_used_for_residualization);
        if (features_tracked[i]->was_used_for_residualization) {
            continue;
        }
        if (features_tracked[i]->is_out_of_view) {
            features_tracked[i]->was_used_for_residualization = true;
            features_to_rezidualize.push_back(features_tracked[i]);
        } else if (features_tracked[i]->positions.size() == parameter->getMaxCameraPoses() + 1) {
            features_tracked[i]->was_used_for_residualization = true;
            features_tracked[i]->positions.pop_back();
            last_camera_pose.decreaseActiveFeaturesCount(features_tracked[i]->feature_id);
            features_to_rezidualize.push_back(features_tracked[i]);
        }
    }

    features_tracked = current_features_tracked;
    pruneCameraPoses();

    for (const std::shared_ptr<FeatureTrack>& feature : features_tracked) {
        if (feature->positions.size() <= 3) {
            continue;
        }
        feature->drawFeatureTrack(frame, cv::Scalar(255, 255, 0), feature->positions.size());
    }
}

void Filter::pruneCameraPoses() {

	int maxTrackNum = 0;
    for (const std::shared_ptr<FeatureTrack>& feature : features_tracked) {
        if (feature->positions.size() > maxTrackNum) {
        	if(!feature->was_used_for_residualization)
        		maxTrackNum = feature->positions.size();
        }
    }
    std::cout << "feature [" << features_tracked.front()->feature_id <<"] tracked: \n";
    for(int i = 0; i < features_tracked.front()->positions.size(); i++) {
    	std::cout << features_tracked.front()->positions[i].transpose() << ", ";
    }
    std::cout << std::endl;

    while(cameraPoseBuf.size() > maxTrackNum) {
    	cameraPoseBuf.deleteOldestCameraPose();
    }

    std::cout << "cameraPoseBuf.end: \n";
    std::set<int>::iterator it;
    for(it=(cameraPoseBuf.end()-1)->feature_ids_.begin();it!=(cameraPoseBuf.end()-1)->feature_ids_.end();it++) {
    	std::cout << *it << ", ";
    }
    std::cout << std::endl;

    std::cout << "maxTrackNUm = " << maxTrackNum << std::endl;
    std::cout << "cameraPoseBuf.size = " << cameraPoseBuf.size() << std::endl;
//    std::size_t size_before = cameraPoseBuf.size();
//
//    std::cout << "residualized_features.size = " << residualized_features.size() << std::endl;
//
//    for (std::size_t i = 0; i < residualized_features.size(); ++i) {
//        CameraPoseBuffer::iterator it = std::end(cameraPoseBuf);
//        it = std::prev(it);
//
//        for (std::size_t j = 0; j < residualized_features[i]->positions.size(); ++j) {
//            it = std::prev(it);
//            it->decreaseActiveFeaturesCount(residualized_features[i]->feature_id);
//        }
//    }
//
//    std::size_t poses_deleted = 0;
//    CameraPoseBuffer& poses = cameraPoseBuf;
//    while (!poses.empty() && poses.front().getActiveFeaturesCount() == 0) {
//        poses.deleteOldestCameraPose();
//        poses_deleted += 1;
//    }
//
//    std::cout << " 🚀 PRUNNING " << poses_deleted << " CAMERA POSES | SIZE BEFORE " << size_before << " | SIZE AFTER " << cameraPoseBuf.size() << std::endl;
//
//    assert(poses.size() <= parameter->getMaxCameraPoses());
}
