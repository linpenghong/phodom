/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:track.h
*   Author  :linpenghong
*   Date    :Jan 23, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_TRACK_H_
#define PHODOM3_INCLUDE_TRACK_H_

#include "feature.h"
#include "feature_track.h"
#include <list>
#include <vector>
#include <memory>

class Filter;

class Track {
	friend class Filter;
public:
	using feature_track_list = std::vector<std::shared_ptr<FeatureTrack>>;
	Track(int track_num);
	Track::feature_track_list processImage(feature_track_list& previous_tracks, cv::Mat& image);
	void drawStats(cv::Mat& image, const std::vector<double>& previous_features_matched,
	            const std::vector<bool>& current_features_matched, const feature_track_list& current_tracks,
	            const std::vector<cv::DMatch>& matches) const;
private:
	void markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks) const;
	void createNewFeatureTracks(std::vector<bool>& feature_matched, feature_track_list& feature_tracks,
		const Feature& frame_features) const;
private:
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    Feature previous_frame_features_;
};

#endif /* PHODOM3_INCLUDE_TRACK_H_ */
