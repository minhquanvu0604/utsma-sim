#include <ros/ros.h>
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




private:
    ros::NodeHandle _nh;

    // ros::Subscriber subOdom_;
    // ros::Subscriber subGoal_;
    // ros::Subscriber subLaser_;

    ros::Subscriber _sub_cone_array;

    ros::Publisher _pub_command_vel;
    ros::Publisher _pub_marker;


};