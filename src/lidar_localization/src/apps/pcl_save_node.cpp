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

std::string FrontPcd_path;

void FrontPoint_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    ros::Time time = cloud_msg->header.stamp;
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*cloud_msg, tmp);
    double tt = time.toSec();
  
    //save to pcd file
    pcl::io::savePCDFileBinary(FrontPcd_path + std::to_string(tt) + ".pcd", tmp);
    pcl::io::savePCDFileASCII(FrontPcd_path + std::to_string(tt) + ".pcd", tmp);
    pcl::io::savePLYFile(FrontPcd_path + std::to_string(tt) + ".ply", tmp);
    
    //save to bin file
    std::ofstream out;
    std::string save_filename = FrontPcd_path + std::to_string(tt) + ".bin";
    out.open(save_filename, std::ios::out | std::ios::binary);
    std::cout << save_filename << " saved" << std::endl;
    int cloudSize = tmp.points.size();
    for (int i = 0; i < cloudSize; ++i)
    {
        float point_x = tmp.points[i].x;
        float point_y = tmp.points[i].y;
        float point_z = tmp.points[i].z;
        out.write(reinterpret_cast<const char *>(&point_x), sizeof(float));
        out.write(reinterpret_cast<const char *>(&point_y), sizeof(float));
        out.write(reinterpret_cast<const char *>(&point_z), sizeof(float));
    }
    out.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_rosbag");
    ROS_INFO("Start Read Rosbag and save it as pcd wiht txt");
    ros::NodeHandle nh;

    std::string front_lidar_topic;

    nh.param("front_lidar_topic", front_lidar_topic, std::string("/lidar/cloud"));
    nh.param("FrontPcd_path", FrontPcd_path, std::string("/home/euvill/Desktop/docker_ws/docker_map"));

    ros::Subscriber sub_cloud1 = nh.subscribe(front_lidar_topic, 100, FrontPoint_callback);
    
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
