#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

namespace lidar_localization {
class ImagePublisher {
  public:
    ImagePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   std::string frame_id,
                   size_t buff_size);
    ImagePublisher() = default;

    void Publish(cv::Mat&  img_ptr_input, double time);
    void Publish(cv::Mat&  img_ptr_input);

    bool HasSubscribers();
  
  private:
    void PublishData(cv::Mat& img, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 