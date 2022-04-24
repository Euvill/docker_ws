#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <mutex>

std::string FrontPcd_path;

std::mutex buff_mutex_;

void FrontPoint_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    ROS_INFO("FrontPoint_callback");
    
    buff_mutex_.lock();

    static unsigned long seq = 0;

    ros::Time time = cloud_msg->header.stamp;
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*cloud_msg, tmp);
    double tt = time.toSec();
  
    pcl::io::savePCDFileBinary(FrontPcd_path + "seq_" + std::to_string(seq) + ".pcd", tmp);

    ++seq;

    buff_mutex_.unlock();
    ROS_INFO("Point Save Success");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_rosbag");
    ROS_INFO("Start Read Rosbag and save it as pcd wiht txt");
    ros::NodeHandle nh;

    std::string front_lidar_topic;

    nh.param("front_lidar_topic", front_lidar_topic, std::string("/lidar/cloud"));
    nh.param("FrontPcd_path", FrontPcd_path, std::string("/home/euvill/Desktop/docker_ws/docker_map/"));

    ros::Subscriber sub_cloud1 = nh.subscribe(front_lidar_topic, 100, FrontPoint_callback);
    
    ros::Rate rate(100);

    while (ros::ok()) {

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
