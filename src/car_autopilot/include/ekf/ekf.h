#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "kb_utils/Encoder.h"
#include "car_autopilot/State.h"

// Jerel's stuff
#include "common_ros/common_ros.h"
#include "common_cpp/common.h"


namespace ekf
{

class EKF
{

public:

  EKF();

private:

  // ROS
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber pose_sub_, ins_sub_, enc_sub_, mocap_sub_;
  ros::Publisher state_pub_;

  // EKF arrays
  Eigen::Matrix<double,5,1> x_;
  Eigen::Matrix<double,5,5> P_, Qx_;
  Eigen::Matrix<double,5,2> B_;
  Eigen::Matrix<double,2,2> Qu_;
  Eigen::Matrix<double,3,5> H_pose_;
  Eigen::Matrix<double,3,3> R_pose_;
  Eigen::Matrix<double,3,5> H_mocap_;
  Eigen::Matrix<double,3,3> R_mocap_;
  Eigen::Matrix<double,5,1> lambda_;
  Eigen::Matrix<double,5,5> Lambda_;

  // additional variables
  double t_prev_;
  double heading_rate_;
  bool is_driving_, okay_to_update_;

  // functions
  void insCallback(const nav_msgs::OdometryConstPtr& msg);
  void encoderCallback(const kb_utils::EncoderConstPtr& msg);
  void poseUpdate(const geometry_msgs::PoseStampedConstPtr& msg);
  void mocapUpdate(const car_autopilot::StateConstPtr& msg);
  void publishState(double u, const kb_utils::EncoderConstPtr& enc_msg);

};

double wrapAngle(double x);


} // namespace ekf

#endif // EKF_H
