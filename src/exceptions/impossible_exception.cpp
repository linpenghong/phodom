/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:impossible_exception.cpp
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/
#include "exceptions/impossible_exception.h"

#include <exception>
#include <string>

ImpossibleException::ImpossibleException(const std::string& msg) {
    msg_ = msg;
}

const char* ImpossibleException::what() const noexcept {
    return msg_.c_str();
}

ImpossibleException::~ImpossibleException() { }
