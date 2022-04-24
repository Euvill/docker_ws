#pragma once

#include <deque>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_localization/models/cloud_filter/box_filter.hpp"

namespace lidar_localization {
class DockerMatching {
  public:
      struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
      };
  public:

    DockerMatching(const YAML::Node& config_node);

    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool GetDockerPose(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose, const Eigen::Matrix4f& laser_pose);

  private:
    bool InitWithConfig(const YAML::Node& config_node);
    bool InitParam(const YAML::Node& config_node);
    bool InitDockerParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);
    float calculateCloudDistance(const CloudData::CLOUD_PTR& cloud1, const CloudData::CLOUD_PTR& cloud2);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;

    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR last_cloud_ptr_;
    
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;

    CloudData::CLOUD_PTR cloud_pattern_ptr_;

    std::vector<float> filter_size_ = {0.0, 0.0, 0.0};

    float ClusterTolerance_;
    int   MinClusterSize_;
    int   MaxClusterSize_;
    float FitnessScore_;
};
}