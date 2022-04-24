#include "lidar_localization/subscriber/laserscan_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization {
LaserScanSubscriber::LaserScanSubscriber(ros::NodeHandle& nh, 
                               std::string topic_name, 
                               size_t buff_size)
    :nh_(nh) {
    //订阅　"/scan" 
    //scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> ("/lidar1/scan", 100, &CloudPublisher::scanCallback, this);
    scan_sub_ = nh.subscribe<sensor_msgs::LaserScan> (topic_name, buff_size, &LaserScanSubscriber::msg_Callback, this);
    //发布LaserScan转换为PointCloud2的后的数据
    //point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void LaserScanSubscriber::msg_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    buff_mutex_.lock();

    sensor_msgs::PointCloud2 scan_msg;

    projector_.transformLaserScanToPointCloud("laser", *scan, scan_msg, tfListener_);

    CloudData scan_data;
    scan_data.time = scan_msg.header.stamp.toSec();
    pcl::fromROSMsg(scan_msg, *(scan_data.cloud_ptr));

    // 测试 cloud_msg 和 cloud_data 数据的正确性
    // sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    // pcl::toROSMsg(*(cloud_data.cloud_ptr), *cloud_ptr_output);
    // point_cloud_publisher_.publish(cloud_msg);

    new_scan_data_.push_back(scan_data);
    
    buff_mutex_.unlock();
}

void LaserScanSubscriber::ParseData(std::deque<CloudData>& scan_data_buff) {
    buff_mutex_.lock();
    if (new_scan_data_.size() > 0) {
        scan_data_buff.insert(scan_data_buff.end(), new_scan_data_.begin(), new_scan_data_.end());
        new_scan_data_.clear();
    }
    buff_mutex_.unlock();
}
}