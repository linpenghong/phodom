/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_track.h
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM2_INCLUDE_FEATURE_TRACK_H_
#define PHODOM2_INCLUDE_FEATURE_TRACK_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <opencv2/core/core.hpp>

class FeatureTrack {
public:
    FeatureTrack();

    const Eigen::Vector2d& operator[] (std::size_t i) const;

    void pushFeaturePosition(double x, double y);
    void popFeaturePosition();

    std::size_t posesTrackedCount() const;

    bool isOutOfView() const;
    void setOutOfView();

    bool wasUsedForResidualization() const;
    void setUsedForResidualization();

    /*
     * for debug
     */
    int getFeatureId() const;
    void drawFeatureTrack(cv::Mat& image, cv::Scalar color, int thickness) const;

private:
    int feature_id_;
    bool is_out_of_view_;
    bool was_used_for_residualization_;
    std::vector<Eigen::Vector2d> positions_;
};

#endif /* PHODOM2_INCLUDE_FEATURE_TRACK_H_ */
