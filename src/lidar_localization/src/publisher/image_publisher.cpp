#include "lidar_localization/publisher/image_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
ImagePublisher::ImagePublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, buff_size);
}

void ImagePublisher::Publish(cv::Mat&  img_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(img_ptr_input, ros_time);
}

void ImagePublisher::Publish(cv::Mat&  img_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(img_ptr_input, time);
}

void ImagePublisher::PublishData(cv::Mat& img, ros::Time time) {
    std_msgs::Header header;
    header.frame_id = frame_id_;
    header.stamp = time;
    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
    publisher_.publish(imgMsg);
}

bool ImagePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} 