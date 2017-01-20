/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_frame.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/
#include "feature_frame.h"
#include "exceptions/general_exception.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>


#define DEBUG

#ifdef DEBUG
#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#endif

FeatureFrame FeatureFrame::detectFeatures(cv::Ptr<cv::FeatureDetector> detector,
        cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat& image) {
	FeatureFrame frame_features;

    assert(image.channels() == 3);

    cv::Mat gray = FeatureFrame::toGray(image);
    frame_features.detectKeypoints(detector, gray);
    frame_features.computeDescriptors(extractor, gray);

    frame_features.drawFeatures(image);

    return frame_features;
}

cv::Mat FeatureFrame::toGray(const cv::Mat& image) {
    switch (image.channels()) {
        case 1:
            return image;
        case 3:
        {
            cv::Mat gray;
            cv::cvtColor(image, gray, CV_BGR2GRAY);
            return gray;
        }
        default:
            throw GeneralException("Unknown image format");
    }
}

void FeatureFrame::detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray) {
     detector->detect(gray, keypoints_);
}

void FeatureFrame::computeDescriptors(cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat gray) {
     extractor->compute(gray, keypoints_, descriptors_);
}

void FeatureFrame::drawFeatures(cv::Mat& image, cv::Scalar color, double scale_factor) {
    int radius = 2;
    for (const cv::KeyPoint& kpt : keypoints_) {
        int x = kpt.pt.x/scale_factor;
        int y = kpt.pt.y/scale_factor;
        cv::line(image, cv::Point(x-radius, y), cv::Point(x+radius, y), color);
        cv::line(image, cv::Point(x, y-radius), cv::Point(x, y+radius), color);
        cv::rectangle(image, cv::Point(x-radius-2, y-radius-2), cv::Point(x+radius+2, y+radius+2), color);
    }

#ifdef DEBUG
    cv::namedWindow( "ORB features", CV_WINDOW_AUTOSIZE );
    cv::imshow( "ORB features", image);
    cv::waitKey(1);
#endif
}

std::vector<cv::DMatch> FeatureFrame::matchFeatures(cv::Ptr<cv::DescriptorMatcher> matcher, const FeatureFrame &other, float threshold) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }

    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_, other.descriptors_, matches);

    //TODO: Move use_homography_filter out to the configuration?
    bool use_homography_filter = true;
    if (use_homography_filter) {
        std::vector<cv::Point2f> this_pts;
        std::vector<cv::Point2f> other_pts;
        this_pts.reserve(matches.size());
        other_pts.reserve(matches.size());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            this_pts.push_back(keypoints_[matches[i].queryIdx].pt);
            other_pts.push_back(other.keypoints_[matches[i].trainIdx].pt);
        }
        cv::Mat good_features_mask;
        cv::Mat H = cv::findHomography(this_pts, other_pts, CV_RANSAC, 3, good_features_mask);

        std::vector<cv::DMatch> good_matches;
        for (std::size_t i = 0; i < matches.size(); ++i) {
            if (good_features_mask.at<bool>(i, 0)) {
                good_matches.push_back(matches[i]);
            }
        }
        return good_matches;
    } else {
        return matches;
    }
}

double FeatureFrame::computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const {
    double min_distance = 100;
    double max_distance = 0;
    double mean_distance = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        mean_distance += match.distance;
        if (match.distance < min_distance) {
            min_distance = match.distance;
        }
        if (match.distance > max_distance) {
            max_distance = match.distance;
        }
    }
    mean_distance /= matches.size();
    return std::max(2*min_distance, 5.0);
}

std::vector<cv::KeyPoint> &FeatureFrame::keypoints() {
    return keypoints_;
}

const std::vector<cv::KeyPoint> &FeatureFrame::keypoints() const {
    return keypoints_;
}
