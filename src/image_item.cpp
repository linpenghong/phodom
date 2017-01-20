/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:image_item.cpp
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/
#include "image_item.h"
#include "exceptions/general_exception.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

//TODO: change camera item from bgr to gray.
ImageItem::ImageItem() : is_valid_(false),  was_processed_(false), time_(0){
}

ImageItem::ImageItem(double time, const cv::Mat& image)
: is_valid_(true){
	time_ = time;
	if(image.channels() == 1) {
		cv::cvtColor(image, image_, CV_GRAY2BGR);
	} else {
		image_ = image;
	}
	was_processed_ = false;
}

ImageItem::operator bool() const {
	return is_valid_;
}

void ImageItem::setProcessed() {
    assert(was_processed_ == false);
    was_processed_ = true;
}

bool ImageItem::wasProcessed() const {
    return was_processed_;
}

double ImageItem::getTime() const {
    return time_;
}

cv::Mat& ImageItem::getImage() {
    return image_;
}

const cv::Mat& ImageItem::getConstImage() const {
    return image_;
}
