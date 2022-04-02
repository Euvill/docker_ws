#include "lidar_localization/models/kalman_filter/kalman_filter.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {

KalmanFilter::KalmanFilter(const YAML::Node& config_node) {
    
    n_g_  = config_node["gyro_noise"].as<double>();
    n_a_  = config_node["acc_noise"].as<double>();
    n_bg_ = config_node["gyro_bias_noise"].as<double>();
    n_ba_ = config_node["acc_bias_noise"].as<double>();
    tau_gb_ = config_node["gyro_tau"].as<double>();
    tau_ga_ = config_node["acc_tau"].as<double>();
    n_p_  = config_node["dp_noise"].as<double>();
    n_phi_= config_node["dphi_noise"].as<double>();

    V_n_g_ = Eigen::Vector3d(n_g_, n_g_, n_g_);
    V_n_a_ = Eigen::Vector3d(n_a_, n_a_, n_a_);
    
    V_n_bg_ = Eigen::Vector3d(n_bg_, n_bg_, n_bg_);
    V_n_ba_ = Eigen::Vector3d(n_ba_, n_ba_, n_ba_);
}

bool KalmanFilter::Init(const Eigen::Matrix4f& laser_odometry, const VelocityData& velocity_data, IMUIntegration &imu_integration) {

    imu_integration.state_.p = laser_odometry.block<3, 1>(0, 3).cast<double>();
    imu_integration.state_.q = Eigen::Quaterniond(laser_odometry.block<3, 3>(0, 0).cast<double>());

    imu_integration.state_.v[0] = velocity_data.linear_velocity.x;
    imu_integration.state_.v[1] = velocity_data.linear_velocity.y;
    imu_integration.state_.v[2] = velocity_data.linear_velocity.z;
    imu_integration.state_.v    = imu_integration.state_.q * imu_integration.state_.v;

    imu_integration.state_.bg = Eigen::Vector3d(0, 0, 0);
    imu_integration.state_.ba = Eigen::Vector3d(0, 0, 0);

    imu_integration.error_state_.x.setZero();
    imu_integration.error_state_.conv.setZero();

    imu_integration.error_state_.conv.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
    imu_integration.error_state_.conv.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
    imu_integration.error_state_.conv.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 3e-4;
    imu_integration.error_state_.conv.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
    imu_integration.error_state_.conv.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-6;
} 

bool KalmanFilter::Predict(const std::deque<IMUData>& current_imu_data, const GNSSData& current_gnss_data, IMUIntegration &imu_integration) {

    Eigen::Vector3d    current_p = imu_integration.state_.p;
    Eigen::Vector3d    current_v = imu_integration.state_.v;
    Eigen::Quaterniond current_q = imu_integration.state_.q;
    //frame relationship: x->E, y->N, z->U 
    Eigen::Vector3d w_ie_n(0, w_ie_ * std::cos(current_gnss_data.latitude * M_PI / 180), w_ie_ * std::sin(current_gnss_data.latitude * M_PI / 180));
    Eigen::Vector3d w_en_n(-current_v[1] / (rm_ + current_gnss_data.altitude), 
                            current_v[0] / (rn_ + current_gnss_data.altitude), 
                            current_v[0] / (rn_ + current_gnss_data.altitude) * std::tan(current_gnss_data.latitude * M_PI / 180));
    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;

    if(current_imu_data.size() > 2) {
        Eigen::Quaterniond Q_bk_to_nk = current_q;     // k   时刻, b-axis 到 n-axis 的旋转四元数
        Eigen::Quaterniond Q_bk_1_to_nk_1(1, 0, 0, 0); // k-1 时刻, b-axis 到 n-axis 的旋转四元数

        Eigen::Vector3d V_n_k = current_v; // k   时刻, n-axis 下的载体速度, END参考系
        Eigen::Vector3d V_n_k_1 = V_n_k;  // k-1 时刻, n-axis 下的载体速度, END参考系
        Eigen::Vector3d delta_v_k(0, 0, 0);
        Eigen::Vector3d delta_v_k_1(0, 0, 0);

        Eigen::Vector3d P_n_k = current_p; // 北向位置, 东向位置, 高程
        for (int i = 2; i < current_imu_data.size(); ++i) {
            // 0. 计算用临时变量求取
            double dt1 = current_imu_data[i-1].time - current_imu_data[i-2].time;//unit: s
            double dt2 =   current_imu_data[i].time - current_imu_data[i-1].time;

            std::vector<Eigen::Vector3d> gyros;
            std::vector<Eigen::Vector3d> accs;

            gyros.push_back(Eigen::Vector3d(current_imu_data[i-2].angular_velocity.x, current_imu_data[i-2].angular_velocity.y, current_imu_data[i-2].angular_velocity.z));
            gyros.push_back(Eigen::Vector3d(current_imu_data[i-1].angular_velocity.x, current_imu_data[i-1].angular_velocity.y, current_imu_data[i-1].angular_velocity.z));
            gyros.push_back(Eigen::Vector3d(current_imu_data[i].angular_velocity.x, current_imu_data[i].angular_velocity.y, current_imu_data[i].angular_velocity.z));

            accs.push_back(Eigen::Vector3d(current_imu_data[i-1].linear_acceleration.x, current_imu_data[i-1].linear_acceleration.y, current_imu_data[i-1].linear_acceleration.z));
            accs.push_back(Eigen::Vector3d(current_imu_data[i].linear_acceleration.x, current_imu_data[i].linear_acceleration.y, current_imu_data[i].linear_acceleration.z));

            V_n_k_1     = V_n_k; // 在更新速度矢量前，保存上一时刻的速度矢量
            delta_v_k_1 = delta_v_k;
            Q_bk_1_to_nk_1 = Q_bk_to_nk;

            // 1. 姿态解算
            Eigen::Quaterniond Q_bk_to_bk_1; // b-axis, k-1 时刻 到 k时刻的旋转四元数
            Eigen::Vector3d q_i_part;
            // 1.1 等效旋转矢量的计算
            Eigen::Vector3d tmpTheta_k_1 = 0.5 * dt1 * (gyros[0] + gyros[1]); //unit: rad
            Eigen::Vector3d tmpTheta_k   = 0.5 * dt2 * (gyros[1] + gyros[2]);
            Eigen::Vector3d phi_k  = tmpTheta_k + 1.0 / 12.0 * tmpTheta_k_1.cross(tmpTheta_k); //载体自身旋转角度, 包括圆锥效应补偿项
            double phi_k_norm = phi_k.norm(); //unit: rad
            if (phi_k_norm == 0) {
                q_i_part = Eigen::Vector3d(0, 0, 0);
            } 
            else {
                q_i_part = std::sin(phi_k_norm / 2.0) * (phi_k / phi_k_norm);
            }
            // 1.2 等效旋转矢量转换为 b-axis, k-1 时刻到 k时刻的旋转四元数
            Q_bk_to_bk_1 = Eigen::Quaterniond(std::cos(phi_k_norm / 2.0), q_i_part[0], q_i_part[1], q_i_part[2]);
            // 1.3 获取 k 时刻, n-axis中的姿态四元数
            Eigen::Vector3d zeta_k = (w_ie_n + w_en_n) * dt2; //地球自转补偿项
            double zeta_k_norm = zeta_k.norm();
            if (zeta_k_norm == 0) {
                q_i_part = Eigen::Vector3d(0, 0, 0);
            } 
            else {
                q_i_part = -std::sin(zeta_k_norm / 2.0) * (zeta_k / zeta_k_norm);
            }
            Eigen::Quaterniond Q_n_k_1_to_n_k(std::cos(zeta_k_norm / 2.0), q_i_part[0], q_i_part[1], q_i_part[2]);
            Q_bk_to_nk = Q_n_k_1_to_n_k * Q_bk_to_nk * Q_bk_to_bk_1;
            Q_bk_to_nk.normalize();

            // 2. 速度解算
            // 2.1 哥氏积分项解算
            Eigen::Vector3d tmp_w = 2 * w_ie_n + w_en_n;
            Eigen::Vector3d a_gc  = gravity_n_ - tmp_w.cross(V_n_k_1);
            Eigen::Vector3d V_cor_k = a_gc * dt2;
            // 2.2 比力积分项解算
            // 2.2.1 等效旋转矢量
            Eigen::Vector3d fs_n_k_1 = accs[0]; 
            Eigen::Vector3d fs_n_k   = accs[1]; 
            // 2.2.2 补偿项
            delta_v_k = 0.5 * (fs_n_k_1 + fs_n_k) * dt2;
            Eigen::Vector3d V_b_f_k = delta_v_k + 0.5 * tmpTheta_k.cross(delta_v_k) + (1.0 / 12.0) * (tmpTheta_k_1.cross(delta_v_k) + delta_v_k_1.cross(tmpTheta_k));


            Eigen::Matrix3d Identity_Matrix = Eigen::Matrix3d::Identity();
            Eigen::Vector3d tmp_zeta = (w_ie_n + w_en_n) * dt2;
            Eigen::Matrix3d zeta_hat;
            zeta_hat <<           0, -tmp_zeta(2), tmp_zeta(1),
                        tmp_zeta(2),            0, tmp_zeta(0),
                        -tmp_zeta(1), tmp_zeta(0),           0;
            Eigen::Vector3d V_n_f_k = (Identity_Matrix - 0.5 * zeta_hat) * Q_bk_1_to_nk_1.toRotationMatrix() * V_b_f_k;
            // 2.3 速度更新
            V_n_k = V_n_k + V_n_f_k + V_cor_k;

            // 3. 位置解算
            P_n_k = P_n_k + 0.5 * (V_n_k_1 + V_n_k) * dt2;

            current_q = Q_bk_to_nk;
            current_v = V_n_k;
            current_p = P_n_k;
        }
    }else {
        std::cout << "IMU buffer error!!!!!!!" << std::endl;
    }

    imu_integration.state_.p = current_p;
    imu_integration.state_.v = current_v;
    imu_integration.state_.q = current_q;

    double delta_t = current_imu_data.back().time - current_imu_data.front().time;
    double delta_t2 = delta_t * delta_t;

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity(); // 定义为 I 矩阵
    Fx.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity() * delta_t;

    Eigen::Matrix<double, 3, 3> f_n = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d f_b(current_imu_data.back().linear_acceleration.x, 
                        current_imu_data.back().linear_acceleration.y,
                        current_imu_data.back().linear_acceleration.z);

    f_b = imu_integration.state_.q.toRotationMatrix() * f_b;
    f_n(0, 1) = -f_b[2]; 
    f_n(0, 2) =  f_b[1];       
    f_n(1, 0) =  f_b[2]; 
    f_n(1, 2) = -f_b[0];      
    f_n(2, 0) = -f_b[1]; 
    f_n(2, 1) =  f_b[0];             
    Fx.block<3, 3>(3, 6)  = f_n * delta_t;
    Fx.block<3, 3>(3, 12) = imu_integration.state_.q.toRotationMatrix() * delta_t;

    Eigen::Matrix<double, 3, 3> w_in_n_hat = Eigen::Matrix<double, 3, 3>::Zero();
    w_in_n_hat(0, 1) = -w_in_n[2]; 
    w_in_n_hat(0, 2) =  w_in_n[1];       
    w_in_n_hat(1, 0) =  w_in_n[2]; 
    w_in_n_hat(1, 2) = -w_in_n[0];      
    w_in_n_hat(2, 0) = -w_in_n[1]; 
    w_in_n_hat(2, 1) =  w_in_n[0]; 
    Fx.block<3, 3>(6, 6)   = Eigen::Matrix<double, 3, 3>::Identity() - w_in_n_hat * delta_t;
    Fx.block<3, 3>(6, 9)   =-Fx.block<3, 3>(3, 12);
    Fx.block<3, 3>(9, 9)   = (1 - delta_t / tau_gb_) * Eigen::Matrix<double, 3, 3>::Identity();
    Fx.block<3, 3>(12, 12) = (1 - delta_t / tau_ga_) * Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 15, 12> Bx = Eigen::Matrix<double, 15, 12>::Zero();
    Bx.block<3, 3>(3, 3)  = Fx.block<3, 3>(3, 12);
    Bx.block<3, 3>(6, 0)  =-Fx.block<3, 3>(3, 12);
    Bx.block<3, 3>(9, 6)  = Fx.block<3, 3>(0, 3);
    Bx.block<3, 3>(12, 9) = Fx.block<3, 3>(0, 3);
    
    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<3, 3>(3, 0)  = Eigen::Matrix<double, 3, 3>::Identity();
    Fi.block<3, 3>(6, 3)  = Eigen::Matrix<double, 3, 3>::Identity();
    Fi.block<3, 3>(9, 6)  = Eigen::Matrix<double, 3, 3>::Identity();
    Fi.block<3, 3>(12, 9) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Identity();
    Qi.block<3, 3>(0, 0) = V_n_g_ * V_n_g_.transpose() * delta_t2;
    Qi.block<3, 3>(3, 3) = V_n_a_ * V_n_a_.transpose() * delta_t2;
    Qi.block<3, 3>(6, 6) = V_n_bg_ * V_n_bg_.transpose() * delta_t;
    Qi.block<3, 3>(9, 9) = V_n_ba_ * V_n_ba_.transpose() * delta_t;

    Eigen::Matrix<double, 12, 1> noise_i;
    noise_i.block<3, 1>(0, 0) = V_n_g_;
    noise_i.block<3, 1>(3, 0) = V_n_a_;
    noise_i.block<3, 1>(6, 0) = V_n_bg_;
    noise_i.block<3, 1>(9, 0) = V_n_ba_;
   
    imu_integration.error_state_.x    = Fx * imu_integration.error_state_.x + Bx * noise_i;
    imu_integration.error_state_.conv = Fx * imu_integration.error_state_.conv * Fx.transpose() + Fi * Qi * Fi.transpose();
    
    return true;
}

bool KalmanFilter::Correct(const Eigen::Matrix4f& laser_odometry, IMUIntegration &imu_integration) {

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    pose.block<3, 3>(0, 0) = imu_integration.state_.q.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = imu_integration.state_.p.cast<float>();

    Eigen::Vector3d delta_p = pose.block<3, 1>(0, 3).cast<double>() - laser_odometry.block<3, 1>(0, 3).cast<double>();
    Eigen::Matrix<double, 3, 3> delta_phi = pose.block<3, 3>(0, 0).cast<double>() * laser_odometry.block<3, 3>(0, 0).cast<double>().transpose();
    delta_phi = delta_phi - Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Vector3d dphi(delta_phi(1, 2), delta_phi(2, 0), delta_phi(0, 1));
    Eigen::Matrix<double, 6, 1> Y;
    Y.head(3) = delta_p;
    Y.tail(3) = dphi;

    Eigen::Matrix<double, 6, 15> Gx = Eigen::Matrix<double, 6, 15>::Zero();
    Gx.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
    Gx.block<3, 3>(3, 6) = Eigen::Matrix<double, 3, 3>::Identity();

    Eigen::Matrix<double, 6, 6> Cx = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> Nu = Eigen::Matrix<double, 6, 6>::Identity();
    Nu.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * n_p_ * n_p_;
    Nu.block<3, 3>(3, 3) = Eigen::Matrix<double, 3, 3>::Identity() * n_phi_ * n_phi_;

    Eigen::Matrix<double, 15, 6> K;
    K = imu_integration.error_state_.conv * Gx.transpose() * (Gx * imu_integration.error_state_.conv * Gx.transpose() + Cx * Nu * Cx.transpose()).inverse();
    imu_integration.error_state_.conv = (Eigen::Matrix<double, 15, 15>::Identity() - K * Gx) * imu_integration.error_state_.conv;
    imu_integration.error_state_.x    = imu_integration.error_state_.x + K * (Y - Gx * imu_integration.error_state_.x);
    
    imu_integration.state_.p = imu_integration.state_.p - imu_integration.error_state_.x.block<3, 1>(0, 0);
    imu_integration.state_.v = imu_integration.state_.v - imu_integration.error_state_.x.block<3, 1>(3, 0);

    Eigen::Vector3d error_phi = imu_integration.error_state_.x.block<3, 1>(6, 0);
    double error_phi_norm = error_phi.norm();
    if (error_phi_norm != 0)
    {
        error_phi = error_phi / error_phi_norm;
        error_phi = error_phi * std::sin(error_phi_norm / 2);
    }
    Eigen::Quaterniond error_phi_q(std::cos(error_phi_norm / 2), error_phi[0], error_phi[1], error_phi[2]);

    imu_integration.state_.q  = error_phi_q * imu_integration.state_.q;
    imu_integration.state_.bg = imu_integration.state_.bg - imu_integration.error_state_.x.block<3, 1>(9, 0);
    imu_integration.state_.ba = imu_integration.state_.ba - imu_integration.error_state_.x.block<3, 1>(12, 0);
    
    imu_integration.error_state_.x.setZero();

    return true;
}
}