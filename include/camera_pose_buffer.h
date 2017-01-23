/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:camera_pose_buffer.h
*   Author  :linpenghong
*   Date    :Jan 12, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_CAMERA_POSE_BUFFER_H_
#define PHODOM_INCLUDE_CAMERA_POSE_BUFFER_H_

#include <boost/circular_buffer.hpp>
#include <utility>

#include "camera_pose.h"

class CameraPoseBuffer {
public:
    using iterator = boost::circular_buffer<CameraPose>::iterator;
    using const_iterator = boost::circular_buffer<CameraPose>::const_iterator;
    using reverse_iterator = boost::circular_buffer<CameraPose>::reverse_iterator;
    using const_reverse_iterator = boost::circular_buffer<CameraPose>::const_reverse_iterator;

    CameraPoseBuffer(int max_camera_poses);

    CameraPose& operator[](std::size_t i);
    const CameraPose& operator[](std::size_t i) const;

    void deleteOldestCameraPose();
    void addNewCameraPose(CameraPose&& pose);

    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;

    reverse_iterator rbegin() noexcept;
    reverse_iterator rend() noexcept;
    const_reverse_iterator rbegin() const noexcept;
    const_reverse_iterator rend() const noexcept;

    CameraPose& front();
    CameraPose& back();

    std::size_t size() const;
    bool empty() const;

private:
    boost::circular_buffer<CameraPose> buffer_;
};

#endif /* PHODOM_INCLUDE_CAMERA_POSE_BUFFER_H_ */
