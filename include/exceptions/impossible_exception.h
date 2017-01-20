/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:impossible_exeption.h
*   Author  :linpenghong
*   Date    :Jan 9, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_EXCEPTIONS_IMPOSSIBLE_EXCEPTION_H_
#define PHODOM_INCLUDE_EXCEPTIONS_IMPOSSIBLE_EXCEPTION_H_


#include <string>

#include "exceptions/base_exception.h"

class ImpossibleException : public BaseException {
public:
    ImpossibleException(const std::string& msg);

    virtual const char* what() const noexcept;

    ~ImpossibleException();

private:
    std::string msg_;
};

#endif /* PHODOM_INCLUDE_EXCEPTIONS_IMPOSSIBLE_EXCEPTION_H_ */
