#include "d_triang_realtime_ros.hpp"


DTRealTimeROSWrapper::DTRealTimeROSWrapper(ros::NodeHandle nh)
    : path_planning_rate(PATH_PLANNING_RATE)
{
    
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

    // plan_one_step(all_visible_cones);

}


void execution_loop(){
    while (ros::ok()){

    }
}


bool DTRealTimeROSWrapper::plan_one_step(const std::vector<Point_2>& cones){
    
    std::cout << "ROSWrapper" << std::endl;

    bool match = match_new_cones(cones);

    ///////////////////////////////////
    if (match)
        std::cout << "MATCHED -------------------------------" << std::endl;
    else    
        std::cout << "NO MATCHED -------------------------------" << std::endl;
    ///////////////////////////////////

    
    if (match) {
        reuse_path();
    }
    else {
        plan_from_scratch();
        // auto edges = get_edges_for_plotting(cones);
        // auto marker = create_triangulation_edge_marker(cones)
    }

    return match;
}





// visualization_msgs::MarkerArray create_triangulation_edge_marker_array(const std::vector<std::pair<Point_2, Point_2>>& edges)
// {
//     visualization_msgs::MarkerArray marker_array;
    
//     // Define marker properties
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "base_footprint"; 
//     marker.ns = "triangulation_edge";
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.type = visualization_msgs::Marker::LINE_STRIP;
//     marker.scale.x = 0.02; // Line width
//     marker.color.r = 1.0; // Red color
//     marker.color.a = 1.0; // Fully opaque

//     // Iterate through the edges and add them to the marker array
//     for (const auto& edge : edges) {
//         geometry_msgs::Point point_start, point_end;
//         point_start.x = edge.first.x();
//         point_start.y = edge.first.y();
//         point_start.z = 0.0; // Assuming 2D points
//         point_end.x = edge.second.x();
//         point_end.y = edge.second.y();
//         point_end.z = 0.0; // Assuming 2D points

//         // Add the start and end points to the marker points
//         marker.points.push_back(point_start);
//         marker.points.push_back(point_end);
//     }

//     marker_array.markers.push_back(marker);

//     return marker_array;
// }


visualization_msgs::Marker DTRealTimeROSWrapper::create_triangulation_edge_marker(const std::vector<std::pair<Point_2, Point_2>>& edges) {
        
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint"; 
    marker.ns = "triangulation_edge";
    // marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.02; // Line width
    marker.color.r = 1.0; // Red color
    marker.color.a = 1.0; // Fully opaque

    // Iterate through the edges and add them to the marker as points
    for (const auto& edge : edges) {
        geometry_msgs::Point point_start, point_end;
        point_start.x = CGAL::to_double(edge.first.x());
        point_start.y = CGAL::to_double(edge.first.y());
        point_start.z = 0.0; // Assuming 2D points
        point_end.x = CGAL::to_double(edge.second.x());
        point_end.y = CGAL::to_double(edge.second.y());
        point_end.z = 0.0; // Assuming 2D points

        // Add the start and end points to the marker points
        marker.points.push_back(point_start);
        marker.points.push_back(point_end);
    }

    return marker;
}
