/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:image_item.h
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM2_INCLUDE_IMAGE_ITEM_H_
#define PHODOM2_INCLUDE_IMAGE_ITEM_H_

#include <string>
#include <opencv2/core/core.hpp>

class ImageItem {
public:
	ImageItem();
	ImageItem(double time, const cv::Mat& image);
	ImageItem(const ImageItem& other) = default;

	ImageItem& operator=(const ImageItem& other) = default;

    operator bool() const;

    void setProcessed();
    bool wasProcessed() const;

    double getTime() const;
    cv::Mat& getImage();
    const cv::Mat& getConstImage() const;

private:
    bool is_valid_;
    double time_;
    cv::Mat image_;
    bool was_processed_;
};

#endif /* PHODOM2_INCLUDE_IMAGE_ITEM_H_ */
