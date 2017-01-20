/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:calibration_exception.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#include "exceptions/calibration_exception.h"
#include <string>

CalibrationFileError::CalibrationFileError(const std::string& msg)
        : BaseException() {
    msg_ = msg;
}

CalibrationFileError::CalibrationFileError(int line_number, const std::string &msg)
        : BaseException() {
    msg_ = "Configuration file error at line " + std::to_string(line_number) + ": " + msg;
}

const char* CalibrationFileError::what() const noexcept {
    return msg_.c_str();
}

CalibrationFileError::~CalibrationFileError() {

}
