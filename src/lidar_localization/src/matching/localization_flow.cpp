#include "lidar_localization/matching/localization_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include <random>
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {
LocalizationFlow::LocalizationFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
    // publisher
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/map", "/lidar", 100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/imu_link", 100);

    matching_ptr_ = std::make_shared<Matching>();
    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/kalman_filter.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

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

    FileManager::CreateDirectory(WORK_SPACE_PATH + "/localization_data");
    FileManager::CreateFile(ground_truth_ofs_, WORK_SPACE_PATH + "/localization_data/ground_truth.txt");
    FileManager::CreateFile(localization_ofs_, WORK_SPACE_PATH + "/localization_data/localization.txt");
}

bool LocalizationFlow::Run() {
    if (matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if (matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());

    if (!ReadData())
        return false;

    if (!InitCalibration()) 
        return false;

    if (!InitGNSS())
        return false;

    if (!InitPose())
        return false;

    while(SyncData(true)) {
        Filter();
        TransformData();
        PublishData();
    }

    return true;
}

bool LocalizationFlow::ReadData() {
    static bool sensor_inited = false;
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    velocity_sub_ptr_->ParseData(velocity_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    if (cloud_data_buff_.empty() || imu_data_buff_.empty()
    || velocity_data_buff_.empty() || gnss_data_buff_.empty())
        return false;

    if (!sensor_inited)
    {
        while (!cloud_data_buff_.empty())
        {
            if (imu_data_buff_.front().time > cloud_data_buff_.front().time
            || velocity_data_buff_.front().time > cloud_data_buff_.front().time
            || gnss_data_buff_.front().time > cloud_data_buff_.front().time)
            {
                cloud_data_buff_.pop_front();
            }
            else
            {
                sensor_inited = true;
                break;
            }
        }
    }

    return sensor_inited;
}

bool LocalizationFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool LocalizationFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data;
        gnss_data.latitude = 48.9826576154;
        gnss_data.longitude = 8.39045533533;
        gnss_data.altitude = 116.39641207;
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool LocalizationFlow::InitPose() {
    static bool pose_inited = false;
    if (pose_inited)
    {
        return true;
    }
    if (!SyncData(false))
    {
        return false;
    }
    TransformData();
    Eigen::Matrix4f laser_odometry;
    laser_odometry = gnss_pose_ * lidar_to_imu_;
    matching_ptr_->SetGNSSPose(laser_odometry);
    matching_ptr_->Update(current_cloud_data_, laser_odometry);
    matching_ptr_->TransformCurrentScan(current_cloud_data_, laser_odometry);
    laser_odometry = laser_odometry * lidar_to_imu_.inverse();
    state_.p = laser_odometry.block<3, 1>(0, 3).cast<double>();
    state_.q = Eigen::Quaterniond(laser_odometry.block<3, 3>(0, 0).cast<double>());
    state_.v[0] = current_velocity_data_.linear_velocity.x;
    state_.v[1] = current_velocity_data_.linear_velocity.y;
    state_.v[2] = current_velocity_data_.linear_velocity.z;
    state_.v = state_.q * state_.v;
    state_.bg = Eigen::Vector3d(0, 0, 0);
    state_.ba = Eigen::Vector3d(0, 0, 0);
    error_state_.x.setZero();
    error_state_.conv.setZero();
    error_state_.conv.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-2;
    error_state_.conv.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1e-3;
    error_state_.conv.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 3e-4;
    error_state_.conv.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e-4;
    error_state_.conv.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e-6;
    pose_inited = true;
    TransformData();
    PublishData();
    return true;
}

bool LocalizationFlow::SyncData(bool inited)
{
    if (cloud_data_buff_.empty())
    {
        return false;
    }
    current_cloud_data_ = cloud_data_buff_.front();
    double sync_time = current_cloud_data_.time;
    while (gnss_data_buff_.size() > 1)
    {
        if (gnss_data_buff_[1].time < sync_time)
        {
            gnss_data_buff_.pop_front();
        }
        else
        {
            break;
        }
    }

    if (gnss_data_buff_.size() > 1)
    {
        GNSSData front_data = gnss_data_buff_.at(0);
        GNSSData back_data = gnss_data_buff_.at(1);
        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        current_gnss_data_.time = sync_time;
        current_gnss_data_.status = back_data.status;
        current_gnss_data_.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
        current_gnss_data_.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
        current_gnss_data_.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
        current_gnss_data_.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        current_gnss_data_.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        current_gnss_data_.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;
    }
    else
    {
        return false;
    }

    while (velocity_data_buff_.size() > 1)
    {
        if (velocity_data_buff_[1].time < sync_time)
        {
            velocity_data_buff_.pop_front();
        }
        else
        {
            break;
        }
    }

    if (velocity_data_buff_.size() > 1)
    {
        VelocityData front_data = velocity_data_buff_.at(0);
        VelocityData back_data = velocity_data_buff_.at(1);

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        current_velocity_data_.time = sync_time;
        current_velocity_data_.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
        current_velocity_data_.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
        current_velocity_data_.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
        current_velocity_data_.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        current_velocity_data_.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        current_velocity_data_.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    }
    else
    {
        return false;
    }

    while (!inited && imu_data_buff_.size() > 1)
    {
        if (imu_data_buff_[1].time < sync_time)
        {
            imu_data_buff_.pop_front();
        }
        else
        {
            break;
        }
    }
    
    if (imu_data_buff_.size() > 1)
    {
        if (!inited)
        {
            current_imu_data_.clear();
            IMUData front_data = imu_data_buff_.at(0);
            IMUData back_data = imu_data_buff_.at(1);
            IMUData synced_data;

            double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
            double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
            synced_data.time = sync_time;
            synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
            synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
            synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
            synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
            synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
            synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
            // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
            // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
            synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
            synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
            synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
            synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
            // 线性插值之后要归一化
            synced_data.orientation.Normlize();
            current_imu_data_.push_back(synced_data);
            imu_data_buff_.pop_front();
            cloud_data_buff_.pop_front();
            return true;
        }

        if (imu_data_buff_.back().time < sync_time)
        {
            return false;
        }
        while (current_imu_data_.size() > 1)
        {
            current_imu_data_.pop_front();
        }
        while (imu_data_buff_.front().time < sync_time)
        {
            IMUData temp = imu_data_buff_.front();
            imu_data_buff_.pop_front();
            current_imu_data_.push_back(temp);
        }
        IMUData front_data = current_imu_data_.back();
        IMUData back_data = imu_data_buff_.at(0);
        IMUData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
        synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
        synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
        // 线性插值之后要归一化
        synced_data.orientation.Normlize();

        current_imu_data_.push_back(synced_data);
        cloud_data_buff_.pop_front();

        return true;
    }
    else
    {
        return false;
    }
}

bool LocalizationFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.back().GetOrientationMatrix();

    VelocityData temp = current_velocity_data_;
    temp.TransformCoordinate(lidar_to_imu_.inverse());
    distortion_adjust_ptr_->SetMotionInfo(0.1, temp);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool LocalizationFlow::PublishData() {
    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = state_.p.cast<float>();
    laser_tf_pub_ptr_->SendTransform(pose, current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(pose, current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());
    SavePose(ground_truth_ofs_, gnss_pose_);
    SavePose(localization_ofs_, pose);
    return true;
}

bool LocalizationFlow::Filter()
{
    Predict();
    Correct();
    return true;
}

bool LocalizationFlow::Predict()
{
    Eigen::Vector3d    current_p = state_.p;
    Eigen::Vector3d    current_v = state_.v;
    Eigen::Quaterniond current_q = state_.q;
    //frame relationship: x->E, y->N, z->U 
    Eigen::Vector3d w_ie_n(0, w_ie_ * std::cos(current_gnss_data_.latitude * M_PI / 180), w_ie_ * std::sin(current_gnss_data_.latitude * M_PI / 180));
    Eigen::Vector3d w_en_n(-current_v[1] / (rm_ + current_gnss_data_.altitude), 
                            current_v[0] / (rn_ + current_gnss_data_.altitude), 
                            current_v[0] / (rn_ + current_gnss_data_.altitude) * std::tan(current_gnss_data_.latitude * M_PI / 180));
    Eigen::Vector3d w_in_n = w_ie_n + w_en_n;


    if(current_imu_data_.size() > 2) {
        Eigen::Quaterniond Q_bk_to_nk = current_q;     // k   时刻, b-axis 到 n-axis 的旋转四元数
        Eigen::Quaterniond Q_bk_1_to_nk_1(1, 0, 0, 0); // k-1 时刻, b-axis 到 n-axis 的旋转四元数

        Eigen::Vector3d V_n_k = current_v; // k   时刻, n-axis 下的载体速度, END参考系
        Eigen::Vector3d V_n_k_1 = V_n_k;  // k-1 时刻, n-axis 下的载体速度, END参考系
        Eigen::Vector3d delta_v_k(0, 0, 0);
        Eigen::Vector3d delta_v_k_1(0, 0, 0);

        Eigen::Vector3d P_n_k = current_p; // 北向位置, 东向位置, 高程
        for (int i = 2; i < current_imu_data_.size(); ++i) {
            // 0. 计算用临时变量求取
            double dt1 = current_imu_data_[i-1].time - current_imu_data_[i-2].time;//unit: s
            double dt2 =   current_imu_data_[i].time - current_imu_data_[i-1].time;

            std::vector<Eigen::Vector3d> gyros;
            std::vector<Eigen::Vector3d> accs;

            gyros.push_back(Eigen::Vector3d(current_imu_data_[i-2].angular_velocity.x, current_imu_data_[i-2].angular_velocity.y, current_imu_data_[i-2].angular_velocity.z));
            gyros.push_back(Eigen::Vector3d(current_imu_data_[i-1].angular_velocity.x, current_imu_data_[i-1].angular_velocity.y, current_imu_data_[i-1].angular_velocity.z));
            gyros.push_back(Eigen::Vector3d(current_imu_data_[i].angular_velocity.x, current_imu_data_[i].angular_velocity.y, current_imu_data_[i].angular_velocity.z));

            accs.push_back(Eigen::Vector3d(current_imu_data_[i-1].linear_acceleration.x, current_imu_data_[i-1].linear_acceleration.y, current_imu_data_[i-1].linear_acceleration.z));
            accs.push_back(Eigen::Vector3d(current_imu_data_[i].linear_acceleration.x, current_imu_data_[i].linear_acceleration.y, current_imu_data_[i].linear_acceleration.z));

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
    state_.p = current_p;
    state_.v = current_v;
    state_.q = current_q;

    double delta_t = current_imu_data_.back().time - current_imu_data_.front().time;
    double delta_t2 = delta_t * delta_t;

    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity(); // 定义为 I 矩阵
    Fx.block<3, 3>(0, 3) = Eigen::Matrix<double, 3, 3>::Identity() * delta_t;

    Eigen::Matrix<double, 3, 3> f_n = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d f_b(current_imu_data_.back().linear_acceleration.x, 
                        current_imu_data_.back().linear_acceleration.y,
                        current_imu_data_.back().linear_acceleration.z);

    f_b = state_.q.toRotationMatrix() * f_b;
    f_n(0, 1) = -f_b[2]; 
    f_n(0, 2) =  f_b[1];       
    f_n(1, 0) =  f_b[2]; 
    f_n(1, 2) = -f_b[0];      
    f_n(2, 0) = -f_b[1]; 
    f_n(2, 1) =  f_b[0];             
    Fx.block<3, 3>(3, 6)  = f_n * delta_t;
    Fx.block<3, 3>(3, 12) = state_.q.toRotationMatrix() * delta_t;

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
   
    error_state_.x = Fx * error_state_.x + Bx * noise_i;
    error_state_.conv = Fx * error_state_.conv * Fx.transpose() + Fi * Qi * Fi.transpose();
    
    return true;
}
bool LocalizationFlow::Correct()
{
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = state_.p.cast<float>();

    Eigen::Matrix4f laser_odometry = pose * lidar_to_imu_;
    // 给NDT地图匹配初值，得到的匹配结果返回给laser_odometry
    matching_ptr_->Update(current_cloud_data_, laser_odometry);
    laser_odometry = laser_odometry * lidar_to_imu_.inverse();

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
    K = error_state_.conv * Gx.transpose() * (Gx * error_state_.conv * Gx.transpose() + Cx * Nu * Cx.transpose()).inverse();
    error_state_.conv = (Eigen::Matrix<double, 15, 15>::Identity() - K * Gx) * error_state_.conv;
    error_state_.x = error_state_.x + K * (Y - Gx * error_state_.x);
    
    state_.p = state_.p - error_state_.x.block<3, 1>(0, 0);
    state_.v = state_.v - error_state_.x.block<3, 1>(3, 0);

    Eigen::Vector3d error_phi = error_state_.x.block<3, 1>(6, 0);
    double error_phi_norm = error_phi.norm();
    if (error_phi_norm != 0)
    {
        error_phi = error_phi / error_phi_norm;
        error_phi = error_phi * std::sin(error_phi_norm / 2);
    }
    Eigen::Quaterniond error_phi_q(std::cos(error_phi_norm / 2), error_phi[0], error_phi[1], error_phi[2]);

    state_.q = error_phi_q * state_.q;
    state_.bg = state_.bg - error_state_.x.block<3, 1>(9, 0);
    state_.ba = state_.ba - error_state_.x.block<3, 1>(12, 0);

    laser_odometry = Eigen::Matrix4f::Identity();
    laser_odometry.block<3, 3>(0, 0) = state_.q.toRotationMatrix().cast<float>();
    laser_odometry.block<3, 1>(0, 3) = state_.p.cast<float>();
    laser_odometry = laser_odometry * lidar_to_imu_;

    matching_ptr_->TransformCurrentScan(current_cloud_data_, laser_odometry);
    
    error_state_.x.setZero();
    return true;
}

bool LocalizationFlow::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);
            
            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }

    return true;
}
}