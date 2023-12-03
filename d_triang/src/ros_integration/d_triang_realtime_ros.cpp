#include "d_triang_realtime_ros.hpp"


DTRealTimeROSWrapper::DTRealTimeROSWrapper(ros::NodeHandle nh) {
    
    ROS_INFO("DTRealTimeROSWrapper Constructor");

    _nh = nh;

    // Subscribing cones location from computervision
    _sub_cone_array = _nh.subscribe("/ground_truth/cones", 1000, &DTRealTimeROSWrapper::cone_array_callback,this);

    // Publishing velocity command
    _pub_command_vel = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out",3,false);


    // Publishing markers
    _pub_marker = _nh.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);
};


void DTRealTimeROSWrapper::cone_array_callback(const eufs_msgs::ConeArrayWithCovariance::ConstPtr& msg){
    
    ROS_INFO("Received a cone ground truth message");

    std::vector<Point_2> all_visible_cones;

    std::vector<eufs_msgs::ConeWithCovariance> yellow_cones = msg->yellow_cones;
    std::vector<eufs_msgs::ConeWithCovariance> blue_cones = msg->blue_cones;
    // std::cout << "Number of yellow cones: " << yellow_cones.size() << std::endl;
    // std::cout << "Number of blue cones: " << blue_cones.size() << std::endl;

    std::vector<eufs_msgs::ConeWithCovariance> combined_cones;
    // Concatenate yellow_cones and blue_cones into combined_cones
    combined_cones.insert(combined_cones.end(), yellow_cones.begin(), yellow_cones.end());
    combined_cones.insert(combined_cones.end(), blue_cones.begin(), blue_cones.end());

    // 'cone' is geometry_msgs::Point 
    for (auto cone : combined_cones){
        Point_2 point_2(cone.point.x,cone.point.y);
        all_visible_cones.push_back(point_2);
    }

    plan_one_step(all_visible_cones);
}
