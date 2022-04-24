#include "lidar_localization/docker/docker_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/tools/file_manager.hpp"


namespace lidar_localization {

DockerFlow::DockerFlow(ros::NodeHandle& nh) {

    scan_sub_ptr_  = std::make_shared<LaserScanSubscriber>(nh, "/scan", 100);

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/lidar/cloud", "/laser", 100);
    
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_localization", "/world", "/laser", 100);
    laser_tf_pub_ptr_   = std::make_shared<TFBroadCaster>("/world", "/laser");
    docker_tf_pub_ptr_  = std::make_shared<TFBroadCaster>("/world", "/docker");

    std::string config_file_path = WORK_SPACE_PATH + "/config/docker/docker.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    docker_matching_ptr_ = std::make_shared<DockerMatching>(config_node);
}

bool DockerFlow::Run() {

    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool DockerFlow::ReadData() {

    scan_sub_ptr_->ParseData(scan_data_buff_);
 
    return true;
}

bool DockerFlow::HasData() {
    if (scan_data_buff_.size() == 0)
        return false;

    return true;
}

bool DockerFlow::ValidData() {

    current_scan_data_ = scan_data_buff_.front();

    scan_data_buff_.pop_front();

    return true;
}

bool DockerFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        docker_matching_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return docker_matching_ptr_->Update(current_scan_data_, lidar_pose_);
    }

    if(docker_matching_ptr_->Update(current_scan_data_, lidar_pose_))
        return true;
    else {
        odometry_inited = false;
        lidar_pose_ = Eigen::Matrix4f::Identity();
        std::cout << "docker localization failed!" << std::endl;
        return true;
    }
}

bool DockerFlow::PublishData() {

    laser_odom_pub_ptr_->Publish(lidar_pose_, current_scan_data_.time); 
    laser_tf_pub_ptr_->SendTransform(lidar_pose_, current_scan_data_.time);

    cloud_pub_ptr_->Publish(current_scan_data_.cloud_ptr, current_scan_data_.time);

    if(docker_matching_ptr_->GetDockerPose(current_scan_data_, docker_pose_, lidar_pose_)) 
        docker_tf_pub_ptr_->SendTransform(docker_pose_, current_scan_data_.time);

    return true;
}

}