/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:general_exception.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#include "exceptions/general_exception.h"
#include "exceptions/base_exception.h"
#include <string>

GeneralException::GeneralException(const std::string& msg)
        : BaseException() {
    msg_ = msg;
}

GeneralException::GeneralException(int line_number, const std::string &msg)
        : BaseException() {
    msg_ = "Configuration file error at line " + std::to_string(line_number) + ": " + msg;
}

const char* GeneralException::what() const noexcept {
    return msg_.c_str();
}

GeneralException::~GeneralException() {

}
