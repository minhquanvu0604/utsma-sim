#include "utsma_navigation/twist_to_ackermann_converter.hpp"

namespace utsma
{
  namespace navigation
  {
    ackermann_msgs::AckermannDrive
    TwistToAckermannConverter::convert(
        const geometry_msgs::Twist twist,
        const double vehicle_length)
    {
      ackermann_msgs::AckermannDrive output;
      const double v = twist.linear.x;
      const double w = twist.angular.z;

      output.acceleration = v * w;

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
      output.steering_angle = steering_angle;
      return output;
    }
  }
}