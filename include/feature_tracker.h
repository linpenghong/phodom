/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_tracker.h
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM2_INCLUDE_FEATURE_TRACKER_H_
#define PHODOM2_INCLUDE_FEATURE_TRACKER_H_

#include "feature_frame.h"
#include "feature_track.h"

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

//for eclipse
#include <bits/shared_ptr.h>

class Filter;

class FeatureTracker {
public:
    friend class Filter;

    using feature_track_list = std::vector<std::shared_ptr<FeatureTrack>>;
    FeatureTracker(int nfeatures_to_track);
    feature_track_list processImage(feature_track_list& previous_tracks, cv::Mat& image);
private:
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    FeatureFrame previous_frame_features_;

    double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;
    void drawStats(cv::Mat& image, const std::vector<double>& previous_features_matched,
            const std::vector<bool>& current_features_matched, const feature_track_list& current_tracks,
            const std::vector<cv::DMatch>& matches) const;
    void markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks) const;
    void createNewFeatureTracks(std::vector<bool>& feature_matched, feature_track_list& feature_tracks,
        const FeatureFrame& frame_features, double scale_factor) const;

};

#endif /* PHODOM2_INCLUDE_FEATURE_TRACKER_H_ */
