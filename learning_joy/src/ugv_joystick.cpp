#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class TeleopUGV
{
public:
  TeleopUGV(ros::NodeHandle nh);

protected:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  ros::NodeHandle nh_;

  int leftThumbUp_ = 1;
  int leftThumbLeft_ = 0;
  int rightThumbUp_ = 4;
  int rightThumbLeft_ = 3;

  double l_scale_, a_scale_;
  ros::Publisher pubBrake_;
  ros::Publisher pubSteering_;
  ros::Publisher pubThrottle_;

  // ros::Publisher vel_pub_;

  ros::Subscriber joy_sub_;
  ros::Publisher ackermann_pub_;
};

TeleopUGV::TeleopUGV(ros::NodeHandle nh)
    : nh_(nh),
      ackermann_pub_(nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out", 10))
{

  // nh_.param("axis_linear", linear_, linear_);
  // nh_.param("axis_angular", angular_, angular_);
  // nh_.param("scale_angular", a_scale_, a_scale_);
  // nh_.param("scale_linear", l_scale_, l_scale_);
  // vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopUGV::joyCallback, this);

  pubBrake_ = nh_.advertise<std_msgs::Float64>("/blue/brake_cmd", 3, false);
  ;
  pubSteering_ = nh_.advertise<std_msgs::Float64>("/blue/steering_cmd", 3, false);
  pubThrottle_ = nh_.advertise<std_msgs::Float64>("/blue/throttle_cmd", 3, false);
}

void TeleopUGV::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  std_msgs::Float64 throttlef;
  std_msgs::Float64 steeringf;
  std_msgs::Float64 brakef;

  // steering: from -10 to 10
  steeringf.data = joy->axes[leftThumbLeft_] * 0.52;
  pubSteering_.publish(steeringf);

  // throttle: from 0 to 0.1
  double throttle = joy->axes[rightThumbUp_] * 0.3;
  // if (throttle < 0)
  //   throttle = 0;
  throttlef.data = throttle;
  pubThrottle_.publish(throttlef);

  // brake: from 0 to 8000
  double brake = -joy->axes[rightThumbUp_] * 8000;
  if (brake < 0)
    brake = 0;
  brakef.data = brake;
  pubBrake_.publish(brakef);

  ROS_INFO_STREAM("steering: " << steeringf.data);
  ROS_INFO_STREAM("throttle: " << throttlef.data);
  ROS_INFO_STREAM("brake: " << brakef.data);

  ackermann_msgs::AckermannDriveStamped drive_stamped;
  drive_stamped.drive.steering_angle = steeringf.data;
  drive_stamped.drive.acceleration = throttle;

  ackermann_pub_.publish(drive_stamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_to_ackermann");
  ros::NodeHandle nh;
  TeleopUGV teleop_turtle(nh);

  ros::spin();
  return 0;
}