/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:base_exception.h
*   Author  :linpenghong
*   Date    :Jan 9, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_EXCEPTIONS_BASE_EXCEPTION_H_
#define PHODOM_INCLUDE_EXCEPTIONS_BASE_EXCEPTION_H_

#include <exception>

class BaseException : public std::exception {
public:
    BaseException();
    virtual const char* what() const noexcept = 0;
};

#endif /* PHODOM_INCLUDE_EXCEPTIONS_BASE_EXCEPTION_H_ */
