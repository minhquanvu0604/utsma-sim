#include "d_triang_color_light_ros.hpp"


DTriangPlannerColorLightROSWrapper::DTriangPlannerColorLightROSWrapper(ros::NodeHandle nh)
    : path_planning_rate(PATH_PLANNING_RATE)
{
    
    ROS_INFO("DTriangPlannerColorLightROSWrapper Constructor");

    _nh = nh;

    // Subscribing cones location from computervision
    _sub_cone_array = _nh.subscribe("/ground_truth/cones", 1000, &DTriangPlannerColorLightROSWrapper::cone_array_callback,this);

    // Publishing velocity command
    _pub_command_vel = _nh.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out",3,false);


    // Publishing markers
    _pub_marker = _nh.advertise<visualization_msgs::MarkerArray>("visualization_marker",3,false);

    execution_loop();
};


void DTriangPlannerColorLightROSWrapper::cone_array_callback(const eufs_msgs::ConeArrayWithCovariance::ConstPtr& msg){
    
    ROS_INFO("Received a cone ground truth message");

    std::vector<Point_2> all_visible_cones;

    std::vector<eufs_msgs::ConeWithCovariance> yellow_cones = msg->yellow_cones;
    std::vector<eufs_msgs::ConeWithCovariance> blue_cones = msg->blue_cones;

    std::vector<DTCL::Cone> cones_in;
    for (const eufs_msgs::ConeWithCovariance& cone : yellow_cones){
        Point_2 point = cone.point;
        DTCL::Cone cone_type(point,1);
        cones_in.push_back(cone_type);
    }
    for (const eufs_msgs::ConeWithCovariance& cone : blue_cones){
        Point_2 point = cone.point;
        DTCL::Cone cone_type(point,0);
        cones_in.push_back(cone_type);
    }

    _incoming_cones = cones_in;
}


void DTriangPlannerColorLightROSWrapper::execution_loop(){
    while (ros::ok()){

        // std::cout << "Loop" << std::endl;
        visualization_msgs::MarkerArray marker_array;


        ros::spinOnce();
        // std::cout << "spinOnce" << std::endl;

        if (_incoming_cones.empty()){
            // std::cout << "Sleeping" << std::endl;
            path_planning_rate.sleep();
            // std::cout << "Next" << std::endl;
            continue;
        }

        // std::cout << "Planning" << std::endl;
        set_cones(_incoming_cones);
        // std::cout << "Planned" << std::endl;

        // Check the number of edges that are going to be plot, if forget to _paths.clear() will create increasing number of markers
        std::vector<std::pair<Point_2, Point_2>> plotting_edges = get_edges_for_plotting();  
        std::cout << "Number of edges for plotting: " << plotting_edges.size() << std::endl;

        visualization_msgs::MarkerArray edges_marker_array = create_triangulation_edge_marker_array(plotting_edges);
        marker_array.markers.insert(marker_array.markers.end(), edges_marker_array.markers.begin(), edges_marker_array.markers.end());

        std::vector<Point_2> best_path = get_ultimate_path(); 
        visualization_msgs::Marker best_path_marker = create_path_marker(best_path, 0.0, 1.0, 0.0, 1);
        marker_array.markers.push_back(best_path_marker);

        _pub_marker.publish(marker_array);

        path_planning_rate.sleep();
    }
}


//////// Visualising //////////////////////////////////
visualization_msgs::MarkerArray DTriangPlannerColorLightROSWrapper::create_triangulation_edge_marker_array(const std::vector<std::pair<Point_2, Point_2>>& edges) {
    visualization_msgs::MarkerArray markers;
    int marker_id = 0;

    for (const auto& edge : edges) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_footprint"; 
        marker.ns = "triangulation_edge";
        marker.id = marker_id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.lifetime = ros::Duration(1/PATH_PLANNING_RATE);
        
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.02; // Line width
        marker.color.r = 1.0; // Red color
        marker.color.a = 1.0; // Fully opaque

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

        markers.markers.push_back(marker);
    }

    return markers;
}

visualization_msgs::Marker DTriangPlannerColorLightROSWrapper::create_path_marker(const std::vector<Point_2>& path, double red, double green, double blue, double alpha) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_footprint";
    marker.ns = "path_marker";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1/PATH_PLANNING_RATE);

    marker.scale.x = 0.02; // Line width
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Iterate through the path and add the points to the marker as points
    for (const auto& point : path) {
        geometry_msgs::Point marker_point;
        marker_point.x = CGAL::to_double(point.x());
        marker_point.y = CGAL::to_double(point.y());
        marker_point.z = 0.0; // Assuming 2D points

        // Add the point to the marker points
        marker.points.push_back(marker_point);
    }

    return marker;
}
