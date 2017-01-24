/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:feature_test.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/
#include "feature_track.h"
#include "track.h"
#include "image_item.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

ros::Time time_beginning_;
ImageItem image_item_;
FeatureTrack featureTrack;
Track featureTracker(60);
Track::feature_track_list features_tracked_;

std::size_t frame_rows_;

void track(double time, cv::Mat& frame) {
	frame_rows_ = frame.rows;
	Track::feature_track_list current_features_tracked;
	current_features_tracked = featureTracker.processImage(features_tracked_, frame);
	features_tracked_ = current_features_tracked;

    int hist[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (const std::shared_ptr<FeatureTrack>& feature : features_tracked_) {
        if (feature->positions.size() < 10) {
            hist[feature->positions.size()] += 1;
        }
    }
    for (const std::shared_ptr<FeatureTrack>& feature : features_tracked_) {
        if (feature->positions.size() <= 3) {
            continue;
        }
        feature->drawFeatureTrack(frame, cv::Scalar(255, 255, 0), feature->positions.size());
    }
}

double getMessageTime(ros::Time stamp) {
    if (time_beginning_.isZero()) {
        time_beginning_ = stamp;
    }
    ros::Duration duration = stamp - time_beginning_;

    return duration.toSec();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	static int counter = 0;
	counter++;
	std::cout << "=====The [" << counter << "th] image reveived!=====\nMsg stamp = "
			<< msg->header.stamp << std::endl;
    try {
        const std_msgs::Header header = msg->header;
        double timestamp = getMessageTime(header.stamp);
        cv::Mat image = cv_bridge::toCvCopy(msg, msg->encoding)->image;
        image_item_ = ImageItem(timestamp, image);
        track(image_item_.getTime(), image_item_.getImage());
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "feature_track_test");
	ros::NodeHandle n;
	ros::Subscriber imageSub = n.subscribe("/wide_stereo/left/image_rect", 5, imageCallback);
	ros::spin();
	return 0;
}
