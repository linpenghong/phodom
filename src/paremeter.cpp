/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:parameter.cpp
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/
#include "exceptions/impossible_exception.h"
#include "exceptions/calibration_exception.h"

#include <boost/filesystem.hpp>
#include <cctype>
#include <fstream>
#include <map>
#include <stdexcept>
#include <iostream>
#include "../include/parameter.h"

Parameter::Parameter()
: body_to_camera_rotation_(Eigen::Quaterniond::Identity()) {
}

void Parameter::setCameraFocalPoint(const Eigen::Vector2d& focal_length) {
    focal_point_ = focal_length;
}

Eigen::Vector2d Parameter::getCameraFocalPoint() const {
    return focal_point_;
}

void Parameter::setCameraOpticalCenter(const Eigen::Vector2d& optical_center) {
    optical_center_ = optical_center;
}

Eigen::Vector2d Parameter::getCameraOpticalCenter() const {
    return optical_center_;
}

void Parameter::setCameraRadialDistortionParams(const Eigen::Vector3d& distortion_params) {
    radial_distortion_ = distortion_params;
}

Eigen::Vector3d Parameter::getCameraRadialDistortionParams() const {
    return radial_distortion_;
}

void Parameter::setCameraTangentialDistortionParams(const Eigen::Vector2d& distortion_params) {
    tangential_distortion_ = distortion_params;
}

Eigen::Vector2d Parameter::getCameraTangentialDistortionParams() const {
    return tangential_distortion_;
}

void Parameter::setCameraDelayTime(double delay_time) {
    camera_delay_time_ = delay_time;
}

double Parameter::getCameraDelayTime() const {
    return camera_delay_time_;
}

void Parameter::setCameraReadoutTime(double readout_time) {
    camera_readout_time_ = readout_time;
}

double Parameter::getCameraReadoutTime() const {
    return camera_readout_time_;
}

double Parameter::getImageNoiseVariance() const {
    return image_noise_variance_;
}

int Parameter::getNumberOfFeaturesToExtract() const {
    return n_features_to_extract_;
}

Eigen::Matrix3d Parameter::getGyroscopeAccelerationSensitivityMatrix() const {
    return gyroscope_acceleration_sensitivity_matrix_;
}

Eigen::Matrix3d Parameter::getGyroscopeShapeMatrix() const {
    return gyroscope_shape_matrix_;
}

Eigen::Matrix3d Parameter::getAccelerometerShapeMatrix() const {
    return accelerometer_shape_matrix_;
}

Eigen::Vector3d Parameter::getGyroscopeBias() const {
    return gyroscope_bias_;
}

Eigen::Vector3d Parameter::getAccelerometerBias() const {
    return accelerometer_bias_;
}

Eigen::Vector3d Parameter::getGlobalGravity() const {
    return global_gravity_;
}

double Parameter::getAccelerometerVariance() const {
    return accelerometer_variance_;
}

double Parameter::getGyroscopeVariance() const {
    return gyroscope_variance_;
}

double Parameter::getAccelerometerRandomWalkVariance() const {
    return accelerometer_random_walk_variance_;
}

double Parameter::getGyroscopeRandomWalkVariance() const {
    return gyroscope_random_walk_variance_;
}

Eigen::Vector3d Parameter::getPositionOfBodyInCameraFrameNoise() const {
    return position_of_body_in_camera_frame_noise_;
}

int Parameter::getMaxCameraPoses() const {
    return max_camera_poses_;
}

int Parameter::getMaxTriangulationIterations() const {
    return max_triangulation_iterations_;
}

Eigen::Vector3d Parameter::getOrientationNoise() const {
    return orientation_noise_;
}

Eigen::Vector3d Parameter::getPositionNoise() const {
    return position_noise_;
}

Eigen::Vector3d Parameter::getVelocityNoise() const {
    return velocity_noise_;
}

Eigen::Vector3d Parameter::getGyroscopeBiasNoise() const {
    return gyroscope_bias_noise_;
}

Eigen::Vector3d Parameter::getAccelerometerBiasNoise() const {
    return accelerometer_bias_noise_;
}

Eigen::Matrix3d Parameter::getGyroscopeAccelerationSensitivityMatrixNoise() const {
    return gyroscope_acceleration_sensitivity_matrix_noise_;
}

Eigen::Matrix3d Parameter::getGyroscopeShapeMatrixNoise() const {
    return gyroscope_shape_matrix_noise_;
}

Eigen::Matrix3d Parameter::getAccelerometerShapeMatrixNoise() const {
    return accelerometer_shape_matrix_noise_;
}

Eigen::Vector2d Parameter::getFocalPointNoise() const {
    return focal_point_noise_;
}

Eigen::Vector2d Parameter::getOpticalCenterNoise() const {
    return optical_center_noise_;
}

Eigen::Vector3d Parameter::getRadialDistortionNoise() const {
    return radial_distortion_noise_;
}

Eigen::Vector2d Parameter::getTangentialDistortionNoise() const {
    return tangential_distortion_noise_;
}

double Parameter::getCameraDelayTimeNoise() const {
    return camera_delay_time_noise_;
}

double Parameter::getCameraReadoutTimeNoise() const {
    return camera_readout_time_noise_;
}

void Parameter::setBodyToCameraRotation(const Eigen::Quaterniond& rotation) {
    body_to_camera_rotation_ = rotation;
}

Eigen::Quaterniond Parameter::getBodyToCameraRotation() const {
    return body_to_camera_rotation_;
}

const std::vector<std::string> Parameter::allowed_params_ = {
        "Camera.focalPoint", "Camera.opticalCenter",
        "Camera.radialDistortion", "Camera.tangentialDistortion",
        "Camera.cameraDelayTime", "Camera.cameraReadoutTime",
        "Camera.imageNoiseVariance",
        "ORBextractor.nFeatures",
        "Imu.Ts", "Imu.Tg", "Imu.Ta", "Imu.gyroscopeBias", "Imu.accelerometerBias", "Imu.globalGravity",
        "Imu.accelerometerVariance", "Imu.gyroscopeVariance",
        "Imu.accelerometerRandomWalkVariance", "Imu.gyroscopeRandomWalkVariance",
		"Filter.maxImuBuffer", "Filter.maxCameraPoses", "Filter.maxTriangulationIterations",
        "Noise.orientation", "Noise.position", "Noise.velocity",
        "Noise.gyroscopeBias", "Noise.accelerometerBias",
        "Noise.Ts", "Noise.Tg", "Noise.Ta",
        "Noise.positionOfBodyInCameraFrame",
        "Noise.focalPoint", "Noise.opticalCenter",
        "Noise.radialDistortion", "Noise.tangentialDistortion",
        "Noise.cameraDelayTime", "Noise.cameraReadoutTime"
};

std::shared_ptr<Parameter> Parameter::fromPath(boost::filesystem::path fname) {
    std::shared_ptr<Parameter> calib = std::make_shared<Parameter>();

    std::ifstream file(fname.c_str());
    if (!file) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    int line_counter = 1;
    std::string line;
    std::map<std::string, std::string> params;
    std::map<std::string, int> param_loc;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            line_counter += 1;
            continue;
        }

        std::size_t delim_pos = line.find_first_of(":");
        if (delim_pos == std::string::npos) {
            throw CalibrationFileError(line_counter, "Missing delimiter.");
        }

        std::string key = line.substr(0, delim_pos);
        std::string value = line.substr(delim_pos + 1, std::string::npos);
        if (params.find(key) != std::end(params)) {
            throw CalibrationFileError(line_counter, "Value already set.");
        }
        if (std::find(std::begin(allowed_params_), std::end(allowed_params_), key) == std::end(allowed_params_)) {
            throw CalibrationFileError(line_counter, "Unknown parameter '" + key + "'.");
        }
        params[key] = value;
        param_loc[key] = line_counter;
        line_counter += 1;
    }
    for (const std::string& allowed_param : allowed_params_) {
        if (params.find(allowed_param) == std::end(params)) {
            throw CalibrationFileError("Parameter " + allowed_param + " not set in configuration file.");
        }
    }

    for (const std::pair<const std::string, std::string>& item : params) {
        std::string key = item.first;
        std::string value = item.second;
        if (key == "Camera.focalPoint") {
            if (!Parameter::tryParseVector2d(value, calib->focal_point_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.opticalCenter") {
            if (!Parameter::tryParseVector2d(value, calib->optical_center_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.radialDistortion") {
            if (!Parameter::tryParseVector3d(value, calib->radial_distortion_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.tangentialDistortion") {
            if (!Parameter::tryParseVector2d(value, calib->tangential_distortion_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.cameraDelayTime") {
            if (!Parameter::tryParseDouble(value, calib->camera_delay_time_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.cameraReadoutTime") {
            if (!Parameter::tryParseDouble(value, calib->camera_readout_time_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.imageNoiseVariance") {
            if (!Parameter::tryParseDouble(value, calib->image_noise_variance_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "ORBextractor.nFeatures") {
            if (!Parameter::tryParseInt(value, calib->n_features_to_extract_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Ts") {
            if (!Parameter::tryParseMatrix3d(value, calib->gyroscope_acceleration_sensitivity_matrix_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Tg") {
            if (!Parameter::tryParseMatrix3d(value, calib->gyroscope_shape_matrix_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Ta") {
            if (!Parameter::tryParseMatrix3d(value, calib->accelerometer_shape_matrix_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.gyroscopeBias") {
            if (!Parameter::tryParseVector3d(value, calib->gyroscope_bias_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.accelerometerBias") {
            if (!Parameter::tryParseVector3d(value, calib->accelerometer_bias_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.globalGravity") {
            if (!Parameter::tryParseVector3d(value, calib->global_gravity_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.accelerometerVariance") {
            if (!Parameter::tryParseDouble(value, calib->accelerometer_variance_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.gyroscopeVariance") {
            if (!Parameter::tryParseDouble(value, calib->gyroscope_variance_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.accelerometerRandomWalkVariance") {
            if (!Parameter::tryParseDouble(value, calib->accelerometer_random_walk_variance_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.gyroscopeRandomWalkVariance") {
            if (!Parameter::tryParseDouble(value, calib->gyroscope_random_walk_variance_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Filter.maxImuBuffer") {
            if (!Parameter::tryParseInt(value, calib->max_imu_buffer_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Filter.maxCameraPoses") {
            if (!Parameter::tryParseInt(value, calib->max_camera_poses_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Filter.maxTriangulationIterations") {
            if (!Parameter::tryParseInt(value, calib->max_triangulation_iterations_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.orientation") {
            if (!Parameter::tryParseVector3d(value, calib->orientation_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.position") {
            if (!Parameter::tryParseVector3d(value, calib->position_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.velocity") {
            if (!Parameter::tryParseVector3d(value, calib->velocity_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.gyroscopeBias") {
            if (!Parameter::tryParseVector3d(value, calib->gyroscope_bias_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.accelerometerBias") {
            if (!Parameter::tryParseVector3d(value, calib->accelerometer_bias_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.Ts") {
            if (!Parameter::tryParseMatrix3d(value, calib->gyroscope_acceleration_sensitivity_matrix_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.Tg") {
            if (!Parameter::tryParseMatrix3d(value, calib->gyroscope_shape_matrix_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.Ta") {
            if (!Parameter::tryParseMatrix3d(value, calib->accelerometer_shape_matrix_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.positionOfBodyInCameraFrame") {
            if (!Parameter::tryParseVector3d(value, calib->position_of_body_in_camera_frame_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.focalPoint") {
            if (!Parameter::tryParseVector2d(value, calib->focal_point_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.opticalCenter") {
            if (!Parameter::tryParseVector2d(value, calib->optical_center_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.radialDistortion") {
            if (!Parameter::tryParseVector3d(value, calib->radial_distortion_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.tangentialDistortion") {
            if (!Parameter::tryParseVector2d(value, calib->tangential_distortion_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.cameraDelayTime") {
            if (!Parameter::tryParseDouble(value, calib->camera_delay_time_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Noise.cameraReadoutTime") {
            if (!Parameter::tryParseDouble(value, calib->camera_readout_time_noise_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else {
            throw ImpossibleException("Non-exhaustive enumeration in camera calibration loading.");
        }
    }

    return calib;
}

bool Parameter::tryParseDouble(const std::string &value, double &out) {
    try {
        std::size_t end = 0;
        out = std::stod(value, &end);
        for (std::size_t pos = end; pos < value.size(); ++pos) {
            if (!std::isspace(value[pos])) {
                return false;
            }
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool Parameter::tryParseInt(const std::string &value, int &out) {
    try {
        std::size_t end = 0;
        out = std::stoi(value, &end);
        for (std::size_t pos = end; pos < value.size(); ++pos) {
            if (!std::isspace(value[pos])) {
                return false;
            }
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool Parameter::tryParseVector2d(const std::string &value, Eigen::Vector2d &out) {
    enum class Expect { OPEN, MAT_ITEM, CLOSE, ONLY_WHITESPACE };
    out = Eigen::Vector2d::Zero();
    Expect expect = Expect::OPEN;
    int item_counter = 0;
    for (std::string::const_iterator it = std::begin(value); it != std::end(value); ++it) {
        if (std::isspace(*it)) {
            continue;
        }
        switch (expect) {
            case Expect::OPEN:
                if (*it != '[')
                    return false;
                expect = Expect::MAT_ITEM;
                break;
            case Expect::MAT_ITEM:
                try {
                    std::size_t after_pos = 0;
                    double item_parsed = stod(std::string(it, std::end(value)), &after_pos);
                    out(item_counter) = item_parsed;
                    it += after_pos - 1;
                    item_counter += 1;
                } catch(...) {
                    return false;
                }
                if (item_counter == 2) {
                    expect = Expect::CLOSE;
                }
                break;
            case Expect::CLOSE:
                if (*it != ']')
                    return false;
                expect = Expect::ONLY_WHITESPACE;
                break;
            case Expect::ONLY_WHITESPACE:
                return false;
                break;
            default:
                throw ImpossibleException("Parameter::tryParseMatrix3d: Non-comprehensive switch statement.");
                break;
        }
    }

    return true;
}

bool Parameter::tryParseVector3d(const std::string &value, Eigen::Vector3d &out) {
    enum class Expect { OPEN, MAT_ITEM, CLOSE, ONLY_WHITESPACE };
    out = Eigen::Vector3d::Zero();
    Expect expect = Expect::OPEN;
    int item_counter = 0;
    for (std::string::const_iterator it = std::begin(value); it != std::end(value); ++it) {
        if (std::isspace(*it)) {
            continue;
        }
        switch (expect) {
            case Expect::OPEN:
                if (*it != '[')
                    return false;
                expect = Expect::MAT_ITEM;
                break;
            case Expect::MAT_ITEM:
                try {
                    std::size_t after_pos = 0;
                    double item_parsed = stod(std::string(it, std::end(value)), &after_pos);
                    out(item_counter) = item_parsed;
                    it += after_pos - 1;
                    item_counter += 1;
                } catch(...) {
                    return false;
                }
                if (item_counter == 3) {
                    expect = Expect::CLOSE;
                }
                break;
            case Expect::CLOSE:
                if (*it != ']')
                    return false;
                expect = Expect::ONLY_WHITESPACE;
                break;
            case Expect::ONLY_WHITESPACE:
                return false;
                break;
            default:
                throw ImpossibleException("Parameter::tryParseMatrix3d: Non-comprehensive switch statement.");
                break;
        }
    }
    return true;
}


bool Parameter::tryParseMatrix3d(const std::string &value, Eigen::Matrix3d &out) {
    enum class Expect { OPEN, MAT_ITEM, LINE_SEP, CLOSE, ONLY_WHITESPACE };
    out = Eigen::Matrix3d::Zero();
    Expect expect = Expect::OPEN;
    int item_counter = 0;
    for (std::string::const_iterator it = std::begin(value); it != std::end(value); ++it) {
        if (std::isspace(*it)) {
            continue;
        }
        switch (expect) {
            case Expect::OPEN:
                if (*it != '[')
                    return false;
                expect = Expect::MAT_ITEM;
                break;
            case Expect::MAT_ITEM:
                try {
                    std::size_t after_pos = 0;
                    double item_parsed = stod(std::string(it, std::end(value)), &after_pos);
                    out(item_counter / 3, item_counter % 3) = item_parsed;
                    it += after_pos - 1;
                    item_counter += 1;
                } catch(...) {
                    return false;
                }
                if (item_counter == 3 || item_counter == 6) {
                    expect = Expect::LINE_SEP;
                } else if (item_counter == 9) {
                    expect = Expect::CLOSE;
                }
                break;
            case Expect::LINE_SEP:
                if (*it != ';')
                    return false;
                expect = item_counter == 9 ? Expect::CLOSE : Expect::MAT_ITEM;
                break;
            case Expect::CLOSE:
                if (*it != ']')
                    return false;
                expect = Expect::ONLY_WHITESPACE;
                break;
            case Expect::ONLY_WHITESPACE:
                return false;
                break;
            default:
                throw ImpossibleException("Parameter::tryParseMatrix3d: Non-comprehensive switch statement.");
                break;
        }
    }

    return true;
}

bool Parameter::tryParseString(const std::string &value, std::string &out) {
    out = value;
    return true;
}
