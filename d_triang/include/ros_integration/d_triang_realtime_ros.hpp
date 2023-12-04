#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eufs_msgs/ConeArrayWithCovariance.h>
#include <geometry_msgs/Point.h>

#include "d_triang_realtime.hpp"
#include "ackermann_msgs/AckermannDriveStamped.h"

/*
The car is in the main thread
*/
class DTRealTimeROSWrapper : public DTRealTime {

public:

    DTRealTimeROSWrapper(ros::NodeHandle nh);

    void cone_array_callback(const eufs_msgs::ConeArrayWithCovariance::ConstPtr& msg);

    void execution_loop();

    bool plan_one_step(const std::vector<Point_2>& cones) override;


private:

    visualization_msgs::Marker create_triangulation_edge_marker(const std::vector<std::pair<Point_2, Point_2>>& edges);



private:

    ros::NodeHandle _nh;

    // ros::Subscriber subOdom_;
    // ros::Subscriber subGoal_;
    // ros::Subscriber subLaser_;

    ros::Subscriber _sub_cone_array;

    ros::Publisher _pub_command_vel;
    ros::Publisher _pub_marker;

    ros::Rate path_planning_rate;

};