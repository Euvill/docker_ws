#include "lidar_localization/docker/docker_matching.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <unordered_map>

#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/registration/icp_m_registration.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"

namespace lidar_localization {
DockerMatching::DockerMatching(const YAML::Node& config_node)
    :local_map_ptr_(new CloudData::CLOUD()),
     cloud_pattern_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig(config_node);
}

bool DockerMatching::InitWithConfig(const YAML::Node& config_node) {

    std::cout << "-----------------地图定位初始化-------------------" << std::endl;

    InitParam(config_node);

    InitDockerParam(config_node);

    InitRegistration(registration_ptr_, config_node);

    InitFilter("local_map", local_map_filter_ptr_, config_node);

    InitFilter("frame", frame_filter_ptr_, config_node);

    return true;
}

bool DockerMatching::InitParam(const YAML::Node& config_node) {

    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool DockerMatching::InitDockerParam(const YAML::Node& config_node) {

    std::cout << "load docker pattern... " << std::endl;
    std::string docker_pattern_path = config_node["docker_pattern_path"].as<std::string>();
    pcl::io::loadPCDFile(docker_pattern_path, *cloud_pattern_ptr_);
    std::cout << "load docker pattern success " << std::endl;

    const YAML::Node Docker_Node = config_node["Docker"];
    const YAML::Node Filter_Node = Docker_Node["voxel_filter"];
    const YAML::Node KdTree_Node = Docker_Node["KdTree"];
    const YAML::Node Match_Node  = Docker_Node["MatchParameter"];

    filter_size_[0] = Filter_Node["leaf_size"][0].as<float>();
    filter_size_[1] = Filter_Node["leaf_size"][1].as<float>();
    filter_size_[2] = Filter_Node["leaf_size"][2].as<float>();

    ClusterTolerance_ = KdTree_Node["ClusterTolerance"].as<float>();
    MinClusterSize_   = KdTree_Node["MinClusterSize"].as<int>();
    MaxClusterSize_   = KdTree_Node["MaxClusterSize"].as<int>();

    FitnessScore_ = Match_Node["FitnessSocre"].as<float>();

    std::cout << "FitnessScore_: " << FitnessScore_ << std::endl;

    return true;
}


bool DockerMatching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "选择的点云匹配方式为：" << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else if (registration_method == "ICP") {
        registration_ptr = std::make_shared<ICPMRegistration>(config_node[registration_method]);
    } 
    else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool DockerMatching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "地图匹配" << filter_user << "选择的滤波方法为：" << filter_mothod << std::endl;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else if (filter_mothod == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool DockerMatching::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    float pose_distance = fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
                          fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
                          fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3));

    float cloud_distance = 0.0;
    if(last_cloud_ptr_ != nullptr)
        cloud_distance = calculateCloudDistance(last_cloud_ptr_, result_cloud_ptr);

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    /*std::cout << "pose , " << current_frame_.pose(0,3) << " , " 
                           << current_frame_.pose(1,3) << " , " 
                           << current_frame_.pose(2,3) << std::endl;
     */
                        
    if (cloud_distance > 500.0) {

        std::cout << "cloud_distance , " << cloud_distance << std::endl;

        step_pose = Eigen::Matrix4f::Identity();
        last_pose = init_pose_;
        predict_pose = init_pose_;
        last_key_frame_pose = init_pose_;

        local_map_ptr_.reset(new CloudData::CLOUD());

        last_cloud_ptr_.reset(new CloudData::CLOUD());

        local_map_frames_.clear();

        return false;
    }

    if (pose_distance > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    last_cloud_ptr_ = result_cloud_ptr;

    return true;
}

float DockerMatching::calculateCloudDistance(const CloudData::CLOUD_PTR& cloud1, const CloudData::CLOUD_PTR& cloud2){
    float distance = 0.0;
    int size = cloud2->size() > cloud1->size() ? cloud1->size() : cloud2->size();
    for(int i = 0; i < size; ++i) {
        distance = distance + fabs(cloud1->at(i).x - cloud2->at(i).x) + fabs(cloud1->at(i).y - cloud2->at(i).y);
    }
    return distance;
}

bool DockerMatching::SetInitPose(const Eigen::Matrix4f& init_pose) {
    
    init_pose_ = init_pose;
    
    return true;
}

bool DockerMatching::UpdateWithNewFrame(const Frame& new_key_frame) {

    Frame key_frame = new_key_frame;
    
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
}

bool DockerMatching::GetDockerPose(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose, const Eigen::Matrix4f& laser_pose) {
    static Eigen::Matrix4f last_cloud_pose = Eigen::Matrix4f::Identity();
    static bool flag = false;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud_data.cloud_ptr);
	vg.setLeafSize(filter_size_[0], filter_size_[1], filter_size_[2]);
	vg.filter(*cloud_filtered);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(ClusterTolerance_); 
	ec.setMinClusterSize(MinClusterSize_);
	ec.setMaxClusterSize(MaxClusterSize_);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

    std::unordered_map<float, Eigen::Vector4f> fitness_centroid_map;
    float min_fitness = 10000.0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

		std::vector<int> indices = it->indices;
		pcl::copyPointCloud(*cloud_filtered, indices, *cloud_cluster);

		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1; // unorganized point cloud dataset
		cloud_cluster->is_dense = true;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_cluster, *cloud_src);

		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud_src);
		icp.setInputTarget(cloud_pattern_ptr_);
		icp.setMaximumIterations(30);
		icp.align(*cloud_src);

		if (icp.hasConverged() && icp.getFitnessScore() < FitnessScore_) {

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            
            fitness_centroid_map[icp.getFitnessScore()] = centroid;

            min_fitness = min_fitness > icp.getFitnessScore() ? icp.getFitnessScore() : min_fitness;
		}
	}

    if (min_fitness != 10000.0) {

        cloud_pose(0, 3) = fitness_centroid_map[min_fitness](0) + laser_pose(0, 3);
        cloud_pose(1, 3) = fitness_centroid_map[min_fitness](1) + laser_pose(1, 3);

        float pose_diff = fabs(last_cloud_pose(0,3) - cloud_pose(0,3)) + 
                          fabs(last_cloud_pose(1,3) - cloud_pose(1,3));

        if (flag && pose_diff > 0.5) {
            std::cout << "no docker TF" << std::endl;
            cloud_pose = last_cloud_pose;
            return false;
        }

        std::cout << "FitnessScore: " << min_fitness << std::endl;
        std::cout << "find docker pose: " << cloud_pose(0, 3) << "," 
                                          << cloud_pose(1, 3) << std::endl;
        
        last_cloud_pose = cloud_pose;

        flag = true;
    }


    return true;
}


}