#include "lidar_localization/matching/matching_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"
#include <random>
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {

MatchingFlow::MatchingFlow(ros::NodeHandle& nh) {
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);

    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);

    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);

    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

    // publisher
    
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/n_frame", 100);

    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/n_frame", 100);

    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/n_frame", 100);

    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/n_frame", "/b_frame", 100);

    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/n_frame", "/b_frame");

    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/n_frame", "/b_frame", 100);

    matching_ptr_ = std::make_shared<Matching>();

    std::string config_file_path = WORK_SPACE_PATH + "/config/matching/kalman_filter.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    InitKalmanFilter(config_node);

    FileManager::CreateDirectory(WORK_SPACE_PATH + "/localization_data");
    FileManager::CreateFile(ground_truth_ofs_, WORK_SPACE_PATH + "/localization_data/ground_truth.txt");
    FileManager::CreateFile(localization_ofs_, WORK_SPACE_PATH + "/localization_data/localization.txt");
}

bool MatchingFlow::InitKalmanFilter(const YAML::Node& config_node) {

    kalman_filter_ptr_ = std::make_shared<KalmanFilter>(config_node);

    return true;
}

bool MatchingFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }
    return calibration_received;
}

bool MatchingFlow::InitGNSS() {
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

bool MatchingFlow::InitPose() {
    static bool pose_inited = false;
    if (pose_inited)
        return true;

    if (!SyncData(false))
        return false;
    
    TransformData();

    Eigen::Matrix4f laser_odometry;
    laser_odometry = gnss_pose_ * lidar_to_imu_;
    
    matching_ptr_->SetGNSSPose(laser_odometry);
    matching_ptr_->Update(current_cloud_data_, laser_odometry);
    matching_ptr_->TransformCurrentScan(current_cloud_data_, laser_odometry);

    laser_odometry = laser_odometry * lidar_to_imu_.inverse();

    kalman_filter_ptr_->Init(laser_odometry, current_velocity_data_, imu_integration);

    pose_inited = true;

    TransformData();

    PublishData();

    return true;
}

bool MatchingFlow::Run() {
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

        kalman_filter_ptr_->Predict(current_imu_data_, current_gnss_data_, imu_integration);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 3>(0, 0) = imu_integration.state_.q.toRotationMatrix().cast<float>();
        pose.block<3, 1>(0, 3) = imu_integration.state_.p.cast<float>();

        Eigen::Matrix4f laser_odometry = pose * lidar_to_imu_; // T_L^n
        matching_ptr_->Update(current_cloud_data_, laser_odometry);
        laser_odometry = laser_odometry * lidar_to_imu_.inverse(); // T_b^n
        kalman_filter_ptr_->Correct(laser_odometry, imu_integration);

        laser_odometry = Eigen::Matrix4f::Identity();
        laser_odometry.block<3, 3>(0, 0) = imu_integration.state_.q.toRotationMatrix().cast<float>();
        laser_odometry.block<3, 1>(0, 3) = imu_integration.state_.p.cast<float>();
        laser_odometry = laser_odometry * lidar_to_imu_; // T_L^n
        matching_ptr_->TransformCurrentScan(current_cloud_data_, laser_odometry);

        TransformData();
        
        PublishData();
    }

    return true;
}

bool MatchingFlow::ReadData() {
    static bool sensor_inited = false;

    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(imu_data_buff_);
    velocity_sub_ptr_->ParseData(velocity_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);

    if (cloud_data_buff_.empty() || imu_data_buff_.empty() || velocity_data_buff_.empty() || gnss_data_buff_.empty())
        return false;

    if (!sensor_inited) {
        while (!cloud_data_buff_.empty()) {
            if (imu_data_buff_.front().time > cloud_data_buff_.front().time || velocity_data_buff_.front().time > cloud_data_buff_.front().time || gnss_data_buff_.front().time > cloud_data_buff_.front().time) {
                cloud_data_buff_.pop_front();
            } else {
                sensor_inited = true;
                break;
            }
        }
    }

    return sensor_inited;
}

bool MatchingFlow::SyncData(bool inited) {
    if (cloud_data_buff_.empty())
        return false;

    current_cloud_data_ = cloud_data_buff_.front();
    double sync_time = current_cloud_data_.time;

    while (gnss_data_buff_.size() > 1) {
        if (gnss_data_buff_[1].time < sync_time)
            gnss_data_buff_.pop_front();
        else
            break;
    }

    if (gnss_data_buff_.size() > 1) {
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
    } else
        return false;

    while (velocity_data_buff_.size() > 1) {
        if (velocity_data_buff_[1].time < sync_time)
            velocity_data_buff_.pop_front();
        else
            break;
    }
    
    if (velocity_data_buff_.size() > 1) {
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
    } else
        return false;

    while (!inited && imu_data_buff_.size() > 1) {
        if (imu_data_buff_[1].time < sync_time)
            imu_data_buff_.pop_front();
        else
            break;
    }
    
    if (imu_data_buff_.size() > 1) {
        if (!inited) {
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
            synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
            synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
            synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
            synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
            synced_data.orientation.Normlize();
            current_imu_data_.push_back(synced_data);
            imu_data_buff_.pop_front();
            cloud_data_buff_.pop_front();
            return true;
        }

        if (imu_data_buff_.back().time < sync_time)
            return false;

        while (current_imu_data_.size() > 1)
            current_imu_data_.pop_front();

        while (imu_data_buff_.front().time < sync_time) {
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
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;

        synced_data.orientation.Normlize();

        current_imu_data_.push_back(synced_data);
        cloud_data_buff_.pop_front();

        return true;
    } else
        return false;
}

bool MatchingFlow::TransformData() {

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

bool MatchingFlow::PublishData() {

    gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = imu_integration.state_.q.toRotationMatrix().cast<float>();
    pose.block<3, 1>(0, 3) = imu_integration.state_.p.cast<float>();
    laser_odom_pub_ptr_->Publish(pose, current_cloud_data_.time); // T_b^n
    laser_tf_pub_ptr_->SendTransform(pose, current_cloud_data_.time); // T_b^n

    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    SavePose(ground_truth_ofs_, gnss_pose_);
    SavePose(localization_ofs_, pose);
    return true;
}

bool MatchingFlow::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose) {
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