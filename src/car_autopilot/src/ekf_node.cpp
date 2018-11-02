#include "ekf/ekf.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_node"); // start ROS node
  ekf::EKF ekf_object; // start callbacks
  ros::spin();
  return 0;
}