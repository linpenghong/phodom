/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature.h
*   Author  :linpenghong
*   Date    :Jan 23, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_FEATURE_H_
#define PHODOM3_INCLUDE_FEATURE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class Feature {
public:
	static Feature detectFeatures(cv::Ptr<cv::FeatureDetector> detector,
			cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat& image);
	std::vector<cv::DMatch> matchFeatures(cv::Ptr<cv::DescriptorMatcher> matcher, const Feature& other,
	            float threshold = 0.1);
	void drawFeatures(cv::Mat& image, cv::Scalar color = cv::Scalar(255, 0, 0), double scale_factor = 1.0);
	std::vector<cv::KeyPoint>& keypoints();
	const std::vector<cv::KeyPoint>& keypoints() const;
	double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;

protected:
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;

	void detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray);
	void computeDescriptors(cv::Ptr<cv::DescriptorExtractor> detector, cv::Mat gray);

	static cv::Mat toGray(const cv::Mat& image);
};

#endif /* PHODOM3_INCLUDE_FEATURE_H_ */
