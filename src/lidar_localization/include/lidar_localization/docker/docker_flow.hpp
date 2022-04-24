#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

// subscriber
#include "lidar_localization/subscriber/laserscan_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/tf_broadcaster.hpp"
#include "lidar_localization/publisher/image_publisher.hpp"

#include "lidar_localization/docker/docker_matching.hpp"

namespace lidar_localization
{
    class DockerFlow
    {
        public:
            DockerFlow(ros::NodeHandle &nh);

            bool Run();
        
        private:
            bool InitPose(); 
            bool ReadData();
            bool HasData();
            bool ValidData(); 
            bool PublishData();

            bool UpdateLaserOdometry();

        private:
            // subscriber
            std::shared_ptr<LaserScanSubscriber> scan_sub_ptr_;
            // publisher
            std::shared_ptr<CloudPublisher>    cloud_pub_ptr_;
            std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
            std::shared_ptr<TFBroadCaster>     laser_tf_pub_ptr_;
            std::shared_ptr<TFBroadCaster>     docker_tf_pub_ptr_;

            std::shared_ptr<DockerMatching> docker_matching_ptr_;

            std::deque<CloudData> scan_data_buff_;

            CloudData current_scan_data_;

            Eigen::Matrix4f lidar_pose_ = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f docker_pose_ = Eigen::Matrix4f::Identity();
    };
}
