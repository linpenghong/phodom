/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:calibration_exception.h
*   Author  :linpenghong
*   Date    :Jan 10, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM_INCLUDE_EXCEPTIONS_CALIBRATION_EXCEPTION_H_
#define PHODOM_INCLUDE_EXCEPTIONS_CALIBRATION_EXCEPTION_H_

#include <string>

#include "exceptions/base_exception.h"

class CalibrationFileError : public BaseException {
public:
    CalibrationFileError(const std::string& msg);
    CalibrationFileError(int line_number, const std::string& msg);

    virtual const char* what() const noexcept;

    virtual ~CalibrationFileError();

private:
    std::string msg_;
};

#endif /* PHODOM_INCLUDE_EXCEPTIONS_CALIBRATION_EXCEPTION_H_ */
