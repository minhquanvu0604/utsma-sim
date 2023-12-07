#include "utsma_navigation/twist_to_ackermann_converter.hpp"

namespace utsma
{
  namespace navigation
  {
    TwistToAckermannConverter::TwistToAckermannConverter(ros::NodeHandle nh)
        : nh_(nh),
          twist_sub_(nh_.subscribe("/cmd_vel", 10, &TwistToAckermannConverter::twistCallback, this)),
          ackermann_pub_(nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/cmd_vel_out", 10))
    {
    }
    TwistToAckermannConverter::~TwistToAckermannConverter()
    {
    }

    void TwistToAckermannConverter::twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
      geometry_msgs::Twist twist = *msg;
      ackermann_pub_.publish(convert(twist, 1.58));
    }
    ackermann_msgs::AckermannDriveStamped
    TwistToAckermannConverter::convert(
        const geometry_msgs::Twist twist,
        const double vehicle_length)
    {
      ackermann_msgs::AckermannDrive drive;
      const double v = twist.linear.x;
      const double w = twist.angular.z;

      drive.acceleration = v * w;

      double steering_angle;
      if (std::abs(w) > 1e-6)
      {
        const double radius = v / w;
        steering_angle = std::atan(vehicle_length / radius);
      }
      else
      {
        steering_angle = 0.0;
      }
      drive.steering_angle = steering_angle;
      ackermann_msgs::AckermannDriveStamped drive_stamped;
      drive_stamped.drive = drive;
      drive_stamped.header.stamp = ros::Time::now();
      return drive_stamped;
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "twist_to_ackermann");
  ros::NodeHandle nh;
  utsma::navigation::TwistToAckermannConverter converter(nh);
  ros::spin();

  return 0;
}