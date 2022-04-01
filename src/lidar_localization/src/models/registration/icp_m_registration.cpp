/*
 * @Description: ICP 匹配模块
 * @Author: euvill
 * @Date: 2021-02-08 21:46:45
 */
#include "lidar_localization/models/registration/icp_m_registration.hpp"

#include "glog/logging.h"

#include <Eigen/Dense>

namespace lidar_localization {

ICPMRegistration::ICPMRegistration(const YAML::Node& node)
    :kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>) {
    
    float max_correspond_dis = node["max_correspondence_distance"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(max_correspond_dis, max_iter);
}

ICPMRegistration::ICPMRegistration(float max_correspond_dis, int max_iter)
    :kdtree_ptr_(new pcl::KdTreeFLANN<CloudData::POINT>){

    SetRegistrationParam(max_correspond_dis, max_iter);
}

bool ICPMRegistration::SetRegistrationParam(float max_correspond_dis, int max_iter) {
    max_correspond_distance_ = max_correspond_dis;
    max_iterator_ = max_iter;

    LOG(INFO) << "ICP Manual 的匹配参数为：" << std::endl
              << "max_correspond_dis: " << max_correspond_dis << ", "
              << "max_iter: " << max_iter
              << std::endl << std::endl;

    return true;
}

bool ICPMRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    // 1. 设置目标点云，用目标点云构建KD树
    target_cloud_.reset(new CloudData::CLOUD);

    target_cloud_ = input_target;

    kdtree_ptr_->setInputCloud(input_target);

    return true;
}

bool ICPMRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                 const Eigen::Matrix4f& predict_pose, 
                                 CloudData::CLOUD_PTR& result_cloud_ptr,
                                 Eigen::Matrix4f& result_pose) {
    // 2. 设置变换矩阵初始值，从变换矩阵中获取旋转矩阵、平移向量
    transformation_  = predict_pose; 
    rotation_matrix_ = transformation_.block<3, 3>(0, 0);  
    translation_     = transformation_.block<3, 1>(0, 3);      
       
    // 3. 计算变换矩阵
    calculateTrans(input_source);
    
    // 4. 使用变换矩阵，对 input_source点云 进行变换，将结果存储到 result_cloud_ptr
    pcl::transformPointCloud(*input_source, *result_cloud_ptr, transformation_);

    // 5. 获取最终变换矩阵的结果
    result_pose = transformation_;

    return true;
}

void ICPMRegistration::calculateTrans(const CloudData::CLOUD_PTR& input_cloud) {
    CloudData::CLOUD_PTR tmp_cloud(new CloudData::CLOUD); // 用于存储每次变换矩阵更新后，点云的变换的结果
    int iter = 0;
    int knn = 1;
    double fitness_score = 0.0;
    while(iter < max_iterator_) {

        pcl::transformPointCloud(*input_cloud, *tmp_cloud, transformation_); // 用最新的变换矩阵，将 输入点云 变换至 tmp_cloud

        Eigen::Matrix<float, 6, 6> Hessian;
        Eigen::Matrix<float, 6, 1> Bias;

        Hessian.setZero();
        Bias.setZero();     

        for(size_t i = 0; i < tmp_cloud->size(); i++) {

            // 1. 获取原始点云数据中的 1 个点
            auto orignal_point = input_cloud->at(i);
            // 2. 判断点云数据是否有效
            if(!pcl::isFinite(orignal_point))
                continue;
            // 3. 获取对应变换后的点
            auto transformed_point = tmp_cloud->at(i);
            // 4. 利用kdtree近邻域搜索，将相近的点存储到 distances、 indexs
            std::vector<float> distances;
            std::vector<int> indexs;
            kdtree_ptr_->nearestKSearch(transformed_point, knn, indexs, distances);
            // 5. 最小距离大于设置的距离阈值，认为该组数据无效
            if(distances[0] > max_correspond_distance_)
                continue;
            // 6. 计算目标点云 transformed_point 中最近点的距离
            Eigen::Vector3f closet_point = Eigen::Vector3f(target_cloud_->at(indexs[0]).x, target_cloud_->at(indexs[0]).y, target_cloud_->at(indexs[0]).z);
            Eigen::Vector3f err_distance = Eigen::Vector3f(transformed_point.x, transformed_point.y, transformed_point.z) - closet_point;
            // 7. 计算 jacobian 矩阵，左乘模型求导
            Eigen::Matrix<float, 3, 6> Jacobian(Eigen::Matrix<float, 3, 6>::Zero());
            Jacobian.leftCols<3>() = Eigen::Matrix3f::Identity();
            Jacobian.rightCols<3>() = -rotation_matrix_ * Sophus::SO3f::hat(Eigen::Vector3f(orignal_point.x, orignal_point.y, orignal_point.z));
            // 8. 计算高斯牛顿方程
            Hessian = Hessian + Jacobian.transpose() * Jacobian;
            Bias = Bias - Jacobian.transpose() * err_distance;
        }
        iter++;
        if(Hessian.determinant() == 0)
            continue;
        // 9. 求解高斯牛顿方差
        Eigen::Matrix<float, 6, 1> delta_x = Hessian.inverse() * Bias;
        // 10. 更新状态变量
        translation_ = translation_ + delta_x.head<3>();
        auto delta_rotation = Sophus::SO3f::exp(delta_x.tail<3>());
        rotation_matrix_ = rotation_matrix_ * delta_rotation.matrix();

        transformation_.block<3, 3>(0, 0) = rotation_matrix_;
        transformation_.block<3, 1>(0, 3) = translation_;


        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        // For each point in the source dataset
        int nr = 0;
        for (size_t i = 0; i < tmp_cloud->size(); ++i)
        {
            auto transformed_point = tmp_cloud->at(i);
            // Find its nearest neighbor in the target
            kdtree_ptr_->nearestKSearch (transformed_point, 1, nn_indices, nn_dists);
            
            // Deal with occlusions (incomplete targets)
            if (nn_dists[0] <= max_correspond_distance_)
            {
                // Add to the fitness score
                fitness_score += nn_dists[0];
                nr++;
            }
        }

        if (nr > 0)
            fitness_score_ = (fitness_score / nr);
        else
            fitness_score_ = (std::numeric_limits<double>::max ());
    }
}    

float ICPMRegistration::GetFitnessScore() {
   return fitness_score_;
}

}