#ifndef UTS_NAVIGATION__TWIST_TO_ACKERMANN_CONVERTER_HPP_
#define UTS_NAVIGATION__TWIST_TO_ACKERMANN_CONVERTER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace utsma
{
  namespace navigation
  {
    class TwistToAckermannConverter
    {
    public:
      static ackermann_msgs::AckermannDrive 
      convert(const geometry_msgs::Twist twist, const double vehicle_length);

    protected:
    };
  }
}

#endif