/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_track.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#include "feature_track.h"
#include "exceptions/general_exception.h"
#include <opencv2/core/core.hpp>

#define DEBUG

#ifdef DEBUG
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#endif

FeatureTrack::FeatureTrack() {
    static int feature_id = 0;
    feature_id_ = feature_id++;

    is_out_of_view_ = false;
    was_used_for_residualization_ = false;
}

const Eigen::Vector2d& FeatureTrack::operator[](std::size_t i) const {
    assert(i < positions_.size() && i >= 0);
    return positions_[i];
}

void FeatureTrack::pushFeaturePosition(double x, double y) {
    Eigen::Vector2d position;
    position << x, y;
    positions_.push_back(std::move(position));
}

void FeatureTrack::popFeaturePosition() {
    assert(positions_.size() > 0);
    positions_.pop_back();
}

std::size_t FeatureTrack::posesTrackedCount() const {
    return positions_.size();
}

bool FeatureTrack::isOutOfView() const {
    return is_out_of_view_;
}

void FeatureTrack::setOutOfView() {
    is_out_of_view_ = true;
}

bool FeatureTrack::wasUsedForResidualization() const {
    return was_used_for_residualization_;
}

void FeatureTrack::setUsedForResidualization() {
    assert(!was_used_for_residualization_);
    was_used_for_residualization_ = true;
}

int FeatureTrack::getFeatureId() const {
    return feature_id_;
}

void FeatureTrack::drawFeatureTrack(cv::Mat& image, cv::Scalar color, int thickness) const {

    if (positions_.size() < 2) {
        return;
    }

    for (std::size_t i = 1; i < positions_.size(); ++i) {
        const Eigen::Vector2d& from = positions_[i-1];
        const Eigen::Vector2d& to = positions_[i];
        cv::line(image, cv::Point(from(0), from(1)), cv::Point(to(0), to(1)), color, 1); //const thickness
    }
#ifdef DEBUG
    cv::namedWindow( "track results", CV_WINDOW_AUTOSIZE );
    cv::imshow( "track results", image);
    cv::waitKey(1);
#endif
}
