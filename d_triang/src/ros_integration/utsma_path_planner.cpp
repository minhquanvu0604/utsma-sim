#include "d_triang_realtime_ros.hpp"


int main(int argc, char **argv)
{
    std::cout<< "STARTING!!" << std::endl;

    ros::init(argc, argv, "utsma_path_planner");

    ros::NodeHandle nh;

    std::shared_ptr<DTRealTimeROSWrapper> utsma_path_planner = std::make_shared<DTRealTimeROSWrapper>(nh);

    ros::spin();

    // ros::shutdown();

  // t.join();

  return 0;
}

