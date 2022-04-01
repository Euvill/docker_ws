#ifndef LIDAR_LOCALIZATION_LOCALIZATION_FLOW_HPP_
#define LIDAR_LOCALIZATION_LOCALIZATION_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"
// models
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_localization/matching/matching.hpp"

namespace lidar_localization
{
    class LocalizationFlow
    {
        public:
            struct State 
            {
                Eigen::Vector3d p;
                Eigen::Vector3d v;
                Eigen::Quaterniond q;
                Eigen::Vector3d bg;
                Eigen::Vector3d ba;
            };
            struct errorState
            {
                Eigen::Matrix<double, 15, 1> x;
                Eigen::Matrix<double, 15, 15> conv;
            };
            LocalizationFlow(ros::NodeHandle &nh);

            bool Run();
        
        private:
            bool ReadData();
            bool InitCalibration();
            bool InitGNSS();
            bool InitPose();
            bool ValidData();
            bool TransformData();
            bool PublishData();
            bool SyncData(bool inited);
            bool Filter();
            bool Predict();
            bool Correct();
            bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);

        private:
            // subscriber
            std::shared_ptr<CloudSubscriber>    cloud_sub_ptr_;
            std::shared_ptr<IMUSubscriber>      imu_sub_ptr_;
            std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
            std::shared_ptr<GNSSSubscriber>     gnss_sub_ptr_;
            std::shared_ptr<TFListener>         lidar_to_imu_ptr_;
            // publisher
            std::shared_ptr<CloudPublisher>    global_map_pub_ptr_;
            std::shared_ptr<CloudPublisher>    local_map_pub_ptr_;
            std::shared_ptr<CloudPublisher>    current_scan_pub_ptr_;
            std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
            std::shared_ptr<TFBroadCaster>     laser_tf_pub_ptr_;
            std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
            // models
            std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;
            std::shared_ptr<Matching> matching_ptr_;

            Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

            std::deque<CloudData>    cloud_data_buff_;
            std::deque<IMUData>      imu_data_buff_;
            std::deque<VelocityData> velocity_data_buff_;
            std::deque<GNSSData>     gnss_data_buff_;

            CloudData           current_cloud_data_;
            std::deque<IMUData> current_imu_data_;
            VelocityData        current_velocity_data_;
            GNSSData            current_gnss_data_;

            Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();

            Eigen::Vector3d V_n_g_;
            Eigen::Vector3d V_n_a_;
            Eigen::Vector3d V_n_bg_;
            Eigen::Vector3d V_n_ba_;

            State state_;
            errorState error_state_;

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
    
            std::ofstream ground_truth_ofs_;
            std::ofstream localization_ofs_;
    };
}

#endif