
#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include <iostream>
#include <memory>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"

namespace lidar_localization {
class LaserScanSubscriber {
      public:
         LaserScanSubscriber(ros::NodeHandle& nh,
                        std::string topic_name,
                        size_t buff_size);
         LaserScanSubscriber() = default;

         void ParseData(std::deque<CloudData>& deque_cloud_data);

      private:
         //订阅 LaserScan 数据，先转换为 PointCloud2，再转换为 pcl::PointCloud
         void msg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan);
      private:
         laser_geometry::LaserProjection projector_;
         tf::TransformListener tfListener_;

         std::deque<CloudData> new_scan_data_;

         //发布　"PointCloud2"
         //ros::Publisher point_cloud_publisher_;

         //订阅 "/scan"
         ros::Subscriber scan_sub_;

         std::mutex buff_mutex_;

         ros::NodeHandle nh_;

         //std::deque<CloudData> new_cloud_data_;
};
}
