/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_track.cpp
*   Author  :linpenghong
*   Date    :Jan 23, 2017
*   Describe:TODO
*
********************************************************/

#include "feature_track.h"
#include <iostream>

#define DEBUG

#ifdef DEBUG
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#endif

FeatureTrack::FeatureTrack() {
    static int static_id = 0;
    feature_id = static_id++;
    is_out_of_view = false;
    was_used_for_residualization = false;
}

void FeatureTrack::drawFeatureTrack(cv::Mat& image, cv::Scalar color, int thickness) const {
    if (positions.size() < 2) {
        return;
    }

    for (std::size_t i = 1; i < positions.size(); ++i) {
        const Eigen::Vector2d& from = positions[i-1];
        const Eigen::Vector2d& to = positions[i];
        cv::line(image, cv::Point(from(0), from(1)), cv::Point(to(0), to(1)), color, 1); //const thickness
    }
#ifdef DEBUG
    cv::namedWindow( "track results", CV_WINDOW_AUTOSIZE );
    cv::imshow( "track results", image);
    cv::waitKey(1);
#endif
}
