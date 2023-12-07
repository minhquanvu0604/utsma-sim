#ifndef UTS_NAVIGATION__TWIST_TO_ACKERMANN_CONVERTER_HPP_
#define UTS_NAVIGATION__TWIST_TO_ACKERMANN_CONVERTER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace utsma
{
  namespace navigation
  {
    class TwistToAckermannConverter
    {
    public:
      TwistToAckermannConverter(ros::NodeHandle nh);
      ~TwistToAckermannConverter();

      static ackermann_msgs::AckermannDriveStamped
      convert(const geometry_msgs::Twist twist, const double vehicle_length);

    protected:
      ros::NodeHandle nh_;
      ros::Subscriber twist_sub_;
      ros::Publisher ackermann_pub_;

      void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
    };
  }
}

#endif