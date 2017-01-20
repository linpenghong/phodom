/********************************************************
*   Copyright (C) 2017 All rights reserved.
*   
*   Filename:parameter.h
*   Author  :linpenghong
*   Date    :Jan 16, 2017
*   Describe:TODO
*
********************************************************/

#ifndef PHODOM3_INCLUDE_PARAMETER_H_
#define PHODOM3_INCLUDE_PARAMETER_H_
#include <boost/filesystem/path.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

class Parameter {
public:
    Parameter();
    Parameter(const Parameter& other) = default;

    Parameter& operator=(const Parameter& other) = default;

    void setCameraFocalPoint(const Eigen::Vector2d& focal_length);
    Eigen::Vector2d getCameraFocalPoint() const;

    void setCameraOpticalCenter(const Eigen::Vector2d& optical_center);
    Eigen::Vector2d getCameraOpticalCenter() const;

    void setCameraRadialDistortionParams(const Eigen::Vector3d& distortion_params);
    Eigen::Vector3d getCameraRadialDistortionParams() const;

    void setCameraTangentialDistortionParams(const Eigen::Vector2d& distortion_params);
    Eigen::Vector2d getCameraTangentialDistortionParams() const;

    void setCameraDelayTime(double delay_time);
    double getCameraDelayTime() const;

    void setCameraReadoutTime(double readout_time);
    double getCameraReadoutTime() const;

    double getImageNoiseVariance() const;

    int getNumberOfFeaturesToExtract() const;

    Eigen::Matrix3d getGyroscopeAccelerationSensitivityMatrix() const;

    Eigen::Matrix3d getGyroscopeShapeMatrix() const;

    Eigen::Matrix3d getAccelerometerShapeMatrix() const;

    Eigen::Vector3d getGyroscopeBias() const;

    Eigen::Vector3d getAccelerometerBias() const;

    Eigen::Vector3d getGlobalGravity() const;

    double getAccelerometerVariance() const;
    double getGyroscopeVariance() const;
    double getAccelerometerRandomWalkVariance() const;
    double getGyroscopeRandomWalkVariance() const;

    int getMaxCameraPoses() const;

    int getMaxTriangulationIterations() const;

    Eigen::Vector3d getOrientationNoise() const;

    Eigen::Vector3d getPositionNoise() const;

    Eigen::Vector3d getVelocityNoise() const;

    Eigen::Vector3d getGyroscopeBiasNoise() const;

    Eigen::Vector3d getAccelerometerBiasNoise() const;

    Eigen::Matrix3d getGyroscopeAccelerationSensitivityMatrixNoise() const;

    Eigen::Matrix3d getGyroscopeShapeMatrixNoise() const;

    Eigen::Matrix3d getAccelerometerShapeMatrixNoise() const;

    Eigen::Vector3d getPositionOfBodyInCameraFrameNoise() const;

    Eigen::Vector2d getFocalPointNoise() const;

    Eigen::Vector2d getOpticalCenterNoise() const;

    Eigen::Vector3d getRadialDistortionNoise() const;

    Eigen::Vector2d getTangentialDistortionNoise() const;

    double getCameraDelayTimeNoise() const;

    double getCameraReadoutTimeNoise() const;

    Eigen::Quaterniond getBodyToCameraRotation() const;

    void setBodyToCameraRotation(const Eigen::Quaterniond& orientation);

    static std::shared_ptr<Parameter> fromPath(boost::filesystem::path fname);

    static bool tryParseInt(const std::string& value, int& out);
    static bool tryParseDouble(const std::string& value, double& out);
    static bool tryParseVector2d(const std::string& value, Eigen::Vector2d& out);
    static bool tryParseVector3d(const std::string& value, Eigen::Vector3d& out);
    static bool tryParseMatrix3d(const std::string& value, Eigen::Matrix3d& out);
    static bool tryParseString(const std::string& value, std::string& out);

    ~Parameter() = default;
protected:
    Eigen::Vector2d focal_point_;
    Eigen::Vector2d optical_center_;
    Eigen::Vector3d radial_distortion_;
    Eigen::Vector2d tangential_distortion_;
    double camera_delay_time_;
    double camera_readout_time_;
    double image_noise_variance_;

    int n_features_to_extract_;

    Eigen::Matrix3d gyroscope_acceleration_sensitivity_matrix_;
    Eigen::Matrix3d gyroscope_shape_matrix_;
    Eigen::Matrix3d accelerometer_shape_matrix_;
    Eigen::Vector3d gyroscope_bias_;
    Eigen::Vector3d accelerometer_bias_;
    Eigen::Vector3d global_gravity_;

    double accelerometer_variance_;
    double gyroscope_variance_;
    double accelerometer_random_walk_variance_;
    double gyroscope_random_walk_variance_;

    int max_imu_buffer_;
    int max_camera_poses_;
    int max_triangulation_iterations_;

    Eigen::Vector3d orientation_noise_;
    Eigen::Vector3d position_noise_;
    Eigen::Vector3d velocity_noise_;
    Eigen::Vector3d gyroscope_bias_noise_;
    Eigen::Vector3d accelerometer_bias_noise_;
    Eigen::Matrix3d gyroscope_acceleration_sensitivity_matrix_noise_;
    Eigen::Matrix3d gyroscope_shape_matrix_noise_;
    Eigen::Matrix3d accelerometer_shape_matrix_noise_;
    Eigen::Vector3d position_of_body_in_camera_frame_noise_;
    Eigen::Vector2d focal_point_noise_;
    Eigen::Vector2d optical_center_noise_;
    Eigen::Vector3d radial_distortion_noise_;
    Eigen::Vector2d tangential_distortion_noise_;
    double camera_delay_time_noise_;
    double camera_readout_time_noise_;

    Eigen::Quaterniond body_to_camera_rotation_;

    static const std::vector<std::string> allowed_params_;
};

#endif /* PHODOM3_INCLUDE_PARAMETER_H_ */
