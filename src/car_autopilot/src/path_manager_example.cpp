#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace car_autopilot
{

path_manager_example::path_manager_example() : path_manager_base()
{
  fil_state_ = fillet_state::STRAIGHT;
  // dub_state_ = dubin_state::FIRST;
}

void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{

  if (num_waypoints_ < 2)
  {
    ROS_WARN_THROTTLE(4, "No waypoints received");
    output.flag = false;
    output.c[0] = 0.0f;
    output.c[1] = 0.0f;
    output.rho = params.R_min;
    output.lambda = 1;
    output.u_d = 0;
  }
  else
  {
      /** Switch the following for flying directly to waypoints, or filleting corners */
      // manage_line(params, input, output);
      manage_fillet(params, input, output);
  }
}

void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{

  Eigen::Vector3f p;
  p << input.pn, input.pe, 0; //Zero altitude

  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

  Eigen::Vector3f w_im1;
  w_im1 << waypoints_[idx_a_].w[0], waypoints_[idx_a_].w[1], 0;
  Eigen::Vector3f w_i;
  w_i << waypoints_[idx_b].w[0], waypoints_[idx_b].w[1], 0;
  Eigen::Vector3f w_ip1;
  w_ip1 << waypoints_[idx_c].w[0], waypoints_[idx_c].w[1], 0;

  output.flag = true;
  output.u_d = waypoints_[idx_a_].u_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  //output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  output.q[0] = q_im1(0);
  output.q[1] = q_im1(1);
  //output.q[2] = q_im1(2);

  Eigen::Vector3f n_i = (q_im1 + q_i).normalized();
  if ((p - w_i).dot(n_i) > 0.0f)
  {
    if (idx_a_ == num_waypoints_ - 1)
      idx_a_ = 0;
    else
      idx_a_++;
  }

}

void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{
  if (num_waypoints_ < 3) //since it fillets don't make sense between just two points
  {
    manage_line(params, input, output);
    return;
  }

  Eigen::Vector3f p;
  p << input.pn, input.pe, 0; //No Altitude

  int idx_b;
  int idx_c;
  if (idx_a_ == num_waypoints_ - 1)
  {
    idx_b = 0;
    idx_c = 1;
  }
  else if (idx_a_ == num_waypoints_ - 2)
  {
    idx_b = num_waypoints_ - 1;
    idx_c = 0;
  }
  else
  {
    idx_b = idx_a_ + 1;
    idx_c = idx_b + 1;
  }

  Eigen::Vector3f w_im1;
  w_im1 << waypoints_[idx_a_].w[0], waypoints_[idx_a_].w[1], 0;
  Eigen::Vector3f w_i;
  w_i << waypoints_[idx_b].w[0], waypoints_[idx_b].w[1], 0;
  Eigen::Vector3f w_ip1;
  w_ip1 << waypoints_[idx_c].w[0], waypoints_[idx_c].w[1], 0;

  float R_min = params.R_min;

  output.u_d = waypoints_[idx_a_].u_d;
  output.r[0] = w_im1(0);
  output.r[1] = w_im1(1);
  //output.r[2] = w_im1(2);
  Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
  Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
  float beta = acosf(-q_im1.dot(q_i));

  Eigen::Vector3f z;
  switch (fil_state_)
  {
  case fillet_state::STRAIGHT:
    output.flag = true;
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    //output.q[2] = q_im1(2);
    output.c[0] = 1;
    output.c[1] = 1;
    //output.c[2] = 1;
    output.rho = 1;
    output.lambda = 1;
    z = w_i - q_im1*(R_min/tanf(beta/2.0));
    if ((p - z).dot(q_im1) > 0)
      fil_state_ = fillet_state::ORBIT;
    break;
  case fillet_state::ORBIT:
    output.flag = false;
    output.q[0] = q_i(0);
    output.q[1] = q_i(1);
    //output.q[2] = q_i(2);
    Eigen::Vector3f c = w_i - (q_im1 - q_i).normalized()*(R_min/sinf(beta/2.0));
    output.c[0] = c(0);
    output.c[1] = c(1);
    //output.c[2] = c(2);
    output.rho = R_min;
    output.lambda = ((q_im1(0)*q_i(1) - q_im1(1)*q_i(0)) > 0 ? 1 : -1);
    z = w_i + q_i*(R_min/tanf(beta/2.0));
    if ((p - z).dot(q_i) > 0)
    {
      if (idx_a_ == num_waypoints_ - 1)
        idx_a_ = 0;
      else
        idx_a_++;
      fil_state_ = fillet_state::STRAIGHT;
    }
    break;
  }
}


Eigen::Matrix3f path_manager_example::rotz(float theta)
{
  Eigen::Matrix3f R;
  R << cosf(theta), -sinf(theta), 0,
  sinf(theta),  cosf(theta), 0,
  0,            0, 1;

  return R;
}

float path_manager_example::mo(float in)
{
  float val;
  if (in > 0)
    val = fmod(in, 2.0*M_PI_F);
  else
  {
    float n = floorf(in/2.0/M_PI_F);
    val = in - n*2.0*M_PI_F;
  }
  return val;
}

}//end namespace
