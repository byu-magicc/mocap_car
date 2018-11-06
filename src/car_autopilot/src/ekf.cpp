#include "ekf/ekf.h"

namespace ekf
{

EKF::EKF() :
  nh_(""),
  nh_private_("~")
{
  // retrieve parameters from ROS parameter server
  common::rosImportMatrix<double>(nh_, "x0", x_);
  common::rosImportMatrix<double>(nh_, "P0", P_);
  common::rosImportMatrix<double>(nh_, "Qx", Qx_);
  common::rosImportMatrix<double>(nh_, "Qu", Qu_);
  common::rosImportMatrix<double>(nh_, "R_pose", R_pose_);
  common::rosImportMatrix<double>(nh_, "R_mocap", R_mocap_);
  common::rosImportMatrix<double>(nh_, "lambda", lambda_);
  Eigen::Matrix<double,5,1> onevec; onevec.setOnes();
  Lambda_ = onevec*lambda_.transpose()+lambda_*onevec.transpose()-lambda_*lambda_.transpose();

  // other parameters and constants
  t_prev_ = 0;
  heading_rate_ = 0;
  is_driving_ = false;
  okay_to_update_ = false;
  B_.setZero();
  B_(3,0) = 1;
  B_(4,1) = 1;
  H_pose_.setZero();
  H_pose_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);
  H_mocap_.setZero();
  H_mocap_.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3);

  // set up ROS subscribers
  enc_sub_ = nh_.subscribe("encoder", 1, &EKF::encoderCallback, this);
  ins_sub_ = nh_.subscribe("ins", 1, &EKF::insCallback, this);
  pose_sub_ = nh_.subscribe("pose", 1, &EKF::poseUpdate, this);
  mocap_sub_ = nh_.subscribe("mocap", 1, &EKF::mocapUpdate, this);

  // set up ROS publishers
  state_pub_ = nh_.advertise<car_autopilot::State>("ekf_state", 1);
}


void EKF::encoderCallback(const kb_utils::EncoderConstPtr& msg)
{
  // get time step
  double t_now = msg->header.stamp.toSec();
  double dt = t_now - t_prev_;
  t_prev_ = t_now;

  // collect measured velocity
  double v_meas = msg->vel;

  // check velocity to start the ekf
  //is_driving_ = false;
  if (!is_driving_)
    if (fabs(v_meas) > 0.002)
      is_driving_ = true;

  // unpack state
  double pn  = x_(0);
  double pe  = x_(1);
  double psi = x_(2);
  double br  = x_(3);
  double bv  = x_(4);

  if (is_driving_)
  {
    // state kinematics
    Eigen::Matrix<double, 5, 1> f;
    f.setZero();
    f(0) = (v_meas+bv)*cos(psi);
    f(1) = (v_meas+bv)*sin(psi);
    f(2) = heading_rate_+br;

    // covariance kinematics
    Eigen::Matrix<double, 5, 5> A;
    A.setZero();
    A(0,2) = -(v_meas+bv)*sin(psi);
    A(0,4) = cos(psi);
    A(1,2) = (v_meas+bv)*cos(psi);
    A(1,4) = sin(psi);
    A(2,3) = 1;

    // propagate the state
    x_ += f*dt;
    P_ = (A*P_+P_*A.transpose()+B_*Qu_*B_.transpose()+Qx_)*dt;

    // allow an update to come through
    // this is needed because ins updates happen at a higher rate than encoder
    okay_to_update_ = true;
  }

  // publish the current state
  publishState(v_meas+bv, msg);
}


void EKF::insCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (is_driving_ && okay_to_update_)
  {
    // extract rotation rate about vertical axis for propagation
    common::Quaternion q_i2b(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
    // Eigen::Matrix3d R_v1_to_b = common::R_v2_to_b(q_i2b.roll())*common::R_v1_to_v2(q_i2b.pitch());
    // Eigen::Vector3d omega_b(msg->twist.twist.angular.x,
    //                         msg->twist.twist.angular.y,
    //                         msg->twist.twist.angular.z);
    // Eigen::Vector3d omega_v1 = R_v1_to_b.transpose()*omega_b;
    // heading_rate_ = omega_v1(2);

    // just use gyro z-axis measurement because using INS attitude estimate is not GPS-denied
    heading_rate_ = msg->twist.twist.angular.z;

    // unpack estimated measurement of position and heading
    Eigen::Vector3d z(msg->pose.pose.position.x,msg->pose.pose.position.y,q_i2b.yaw());
    Eigen::Vector3d hx = x_.segment<3>(0);

    // residual error
    Eigen::Vector3d r = z-hx;
    r(2) = wrapAngle(r(2)); // make sure to use shortest angle in heading update

    // apply update
    Eigen::Matrix<double,5,3> K = P_*H_pose_.transpose()*(H_pose_*P_*H_pose_.transpose()+R_pose_).inverse();
    x_ += lambda_.cwiseProduct(K*r);
    P_ -= Lambda_.cwiseProduct(K*H_pose_*P_);

    // don't allow more updates before propagation happens
    okay_to_update_ = false;
    std::cout << "\n\n" << r << "\n\n";
  }
}


void EKF::poseUpdate(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (is_driving_ && okay_to_update_)
  {
    // extract heading from quaternion
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    double psi_meas = tf::getYaw(pose.getRotation());

    // unpack estimated measurement
    Eigen::Vector3d z(msg->pose.position.x,msg->pose.position.y,psi_meas);
    Eigen::Vector3d hx = x_.segment<3>(0);

    // residual error
    Eigen::Vector3d r = z-hx;

    // make sure to use shortest angle in heading update
    r(2) = wrapAngle(r(2));

    // apply update
    Eigen::Matrix<double,5,3> K = P_*H_pose_.transpose()*(H_pose_*P_*H_pose_.transpose()+R_mocap_).inverse();
    x_ += lambda_.cwiseProduct(K*r);
    P_ -= Lambda_.cwiseProduct(K*H_pose_*P_);

    // don't allow more updates before propagation happens
    okay_to_update_ = false;
  }
}

void EKF::mocapUpdate(const car_autopilot::StateConstPtr& msg)
{
  if (is_driving_ && okay_to_update_)
  {

    // unpack estimated measurement
    Eigen::Vector3d z(msg->p_north,msg->p_east,msg->psi);
    Eigen::Vector3d hx = x_.segment<3>(0);

    // residual error
    Eigen::Vector3d r = z-hx;

    // make sure to use shortest angle in heading update
    r(2) = wrapAngle(r(2));


    // apply update
    Eigen::Matrix<double,5,3> K = P_*H_mocap_.transpose()*(H_mocap_*P_*H_mocap_.transpose()+R_mocap_).inverse();
    x_ += lambda_.cwiseProduct(K*r);
    P_ -= Lambda_.cwiseProduct(K*H_mocap_*P_);


    // don't allow more updates before propagation happens
    okay_to_update_ = false;
  }
}


void EKF::publishState(double u, const kb_utils::EncoderConstPtr& enc_msg)
{
  car_autopilot::State msg;
  msg.header.stamp = enc_msg->header.stamp;
  msg.p_north = x_(0); // north position (m)
  msg.p_east =  x_(1); // east position (m)
  msg.psi =     x_(2); // unwrapped yaw angle (rad)
  msg.b_r =     x_(3); // heading rate bias
  msg.b_u =     x_(4); // velocity bias
  msg.u =       u;     // body fixed forward velocity (m/s)

  msg.psi_deg = wrapAngle(x_(2))*180/M_PI; // unwrapped yaw angle (deg)
  state_pub_.publish(msg);
}


double wrapAngle(double x)
{
    x = fmod(x + M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}


} // namespace ekf
