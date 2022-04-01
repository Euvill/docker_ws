#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_M_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_M_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <sophus/se3.hpp>

namespace lidar_localization {
class ICPMRegistration: public RegistrationInterface {
  public:
    ICPMRegistration(const YAML::Node& node);
    ICPMRegistration(float max_correspond_dis, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
    float GetFitnessScore() override;
  
  private:
    bool SetRegistrationParam(float max_correspond_dis, int max_iter);
    void calculateTrans(const CloudData::CLOUD_PTR& input_cloud);

  private:
    CloudData::CLOUD_PTR target_cloud_;
    pcl::KdTreeFLANN<CloudData::POINT>::Ptr kdtree_ptr_;
    
    float max_correspond_distance_;      // 阈值
    int max_iterator_;                   // 最大迭代次数
    
    double fitness_score_ = 0.0;

    Eigen::Matrix3f  rotation_matrix_;   // 旋转矩阵
    Eigen::Vector3f  translation_;       // 平移矩阵
    Eigen::Matrix4f  transformation_;    // 转换矩阵     
};
}

#endif