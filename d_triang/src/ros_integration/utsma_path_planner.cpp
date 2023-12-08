#include "d_triang_color_light_ros.hpp"

int main(int argc, char **argv)
{
    std::cout<< "STARTING!!" << std::endl;

    ros::init(argc, argv, "utsma_path_planner");

    ros::NodeHandle nh;

    std::shared_ptr<DTriangPlannerColorLightROSWrapper> utsma_path_planner = std::make_shared<DTriangPlannerColorLightROSWrapper>(nh);

    ros::spin();

    // ros::shutdown();

  // t.join();

  return 0;
}

