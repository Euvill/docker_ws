#pragma once

#include <ros/ros.h>
#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/imu_data.hpp"
#include "lidar_localization/sensor_data/gnss_data.hpp"
#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class KalmanFilter {
public:
    struct State {
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Quaterniond q;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
    };

    struct errorState {
        Eigen::Matrix<double, 15, 1> x;
        Eigen::Matrix<double, 15, 15> conv;
    };

    struct IMUIntegration {
        double T;
        State state_;
        errorState error_state_;
    };

    KalmanFilter(const YAML::Node& config_node);
    bool Init(const Eigen::Matrix4f& laser_odometry, const VelocityData& velocity_data, IMUIntegration &imu_integration);
    bool Predict(const std::deque<IMUData>& current_imu_data, const GNSSData& current_gnss_data, IMUIntegration &imu_integration);
    bool Correct(const Eigen::Matrix4f& laser_odometry, IMUIntegration &imu_integration);

private:
    Eigen::Vector3d V_n_g_;
    Eigen::Vector3d V_n_a_;
    Eigen::Vector3d V_n_bg_;
    Eigen::Vector3d V_n_ba_;

    double n_g_; // 陀螺仪测量噪声
    double n_a_; // 加速度计测量噪声
    double n_bg_;// 陀螺仪零偏噪声
    double n_ba_;// 加速度计零偏噪声

    double n_p_; // 激光雷达位置测量噪声
    double n_phi_; // 激光雷达失准角测量噪声

    Eigen::Vector3d gravity_n_ = Eigen::Vector3d(0.0, 0.0, -9.809432933396721);
    double w_ie_ = 7.27220521664304e-05; // rad/s
    double rm_   = 6371829.37587;
    double rn_   = 6390325.45972;
    double tau_gb_;
    double tau_ga_;
};
}