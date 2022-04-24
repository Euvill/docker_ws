#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/docker/docker_flow.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "docker_node");
    ros::NodeHandle nh;

    std::shared_ptr<DockerFlow> docker_flow_ptr = std::make_shared<DockerFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        docker_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}