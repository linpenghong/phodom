/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_track.h
*   Author  :linpenghong
*   Date    :Jan 23, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_FEATURE_TRACK_H_
#define PHODOM3_INCLUDE_FEATURE_TRACK_H_

#include <vector>
#include <eigen3/Eigen/Core>
#include <opencv2/core/core.hpp>

class FeatureTrack {
public:
	FeatureTrack();
	const Eigen::Vector2d& operator[] (std::size_t i) const;
	void drawFeatureTrack(cv::Mat& image, cv::Scalar color, int thickness) const;

public:
    int feature_id;
    bool is_out_of_view;
    bool was_used_for_residualization;
    std::vector<Eigen::Vector2d> positions;
};

#endif /* PHODOM3_INCLUDE_FEATURE_TRACK_H_ */
