/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:general_exception.h
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_EXCEPTIONS_GENERAL_EXCEPTION_H_
#define PHODOM_INCLUDE_EXCEPTIONS_GENERAL_EXCEPTION_H_

#include <string>

#include "exceptions/base_exception.h"

class GeneralException : public BaseException {
public:
    GeneralException(const std::string& msg);
    GeneralException(int line_number, const std::string& msg);

    virtual const char* what() const noexcept;

    virtual ~GeneralException();

private:
    std::string msg_;
};

#endif /* PHODOM_INCLUDE_EXCEPTIONS_GENERAL_EXCEPTION_H_ */
