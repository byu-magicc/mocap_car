#include "path_follower_base.h"
#include "path_follower_example.h"

namespace car_autopilot
{

path_follower_base::path_follower_base():
  nh_(ros::NodeHandle()),
  nh_private_(ros::NodeHandle("~"))
{
  vehicle_state_sub_ = nh_.subscribe<car_autopilot::State>("state", 1, &path_follower_base::vehicle_state_callback, this);
  current_path_sub_ = nh_.subscribe<car_autopilot::Current_Path>("current_path", 1,
                      &path_follower_base::current_path_callback, this);

  nh_private_.param<double>("CHI_INFTY", params_.psi_infty, 1.0472);
  nh_private_.param<double>("K_PATH", params_.k_path, 0.025);
  nh_private_.param<double>("K_ORBIT", params_.k_orbit, 4.0);

//  func_ = boost::bind(&path_follower_base::reconfigure_callback, this, _1, _2);
//  server_.setCallback(func_);

  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_follower_base::update, this);
  controller_commands_pub_ = nh_.advertise<car_autopilot::Controller_Commands>("controller_commands", 1);

  state_init_ = false;
  current_path_init_ = false;
}

void path_follower_base::update(const ros::TimerEvent &)
{

  struct output_s output;

  if (state_init_ == true && current_path_init_ == true)
  {
    follow(params_, input_, output);
    car_autopilot::Controller_Commands msg;
    msg.psi_c = output.psi_c;
    msg.u_c = output.u_c;
    controller_commands_pub_.publish(msg);
  }
}

void path_follower_base::vehicle_state_callback(const car_autopilot::StateConstPtr &msg)
{
  input_.pn = msg->p_north;               /** position north */
  input_.pe = msg->p_east;               /** position east */
  input_.psi = msg->psi;
  input_.u = msg->u;

  state_init_ = true;
}

void path_follower_base::current_path_callback(const car_autopilot::Current_PathConstPtr &msg)
{
  if (msg->path_type == msg->LINE_PATH)
    input_.p_type = path_type::Line;
  else if (msg->path_type == msg->ORBIT_PATH)
    input_.p_type = path_type::Orbit;

  input_.u_d = msg->Va_d;

  for (int i = 0; i < 3; i++)
  {
    input_.r_path[i] = msg->r[i];
    input_.q_path[i] = msg->q[i];
    input_.c_orbit[i] = msg->c[i];
  }
  input_.rho_orbit = msg->rho;
  input_.lam_orbit = msg->lambda;
  current_path_init_ = true;
}

//void path_follower_base::reconfigure_callback(car_autopilot::FollowerConfig &config, uint32_t level)
//{
//  params_.chi_infty = config.CHI_INFTY;
//  params_.k_path = config.K_PATH;
//  params_.k_orbit = config.K_ORBIT;
//}
} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_follower");
  car_autopilot::path_follower_base *path = new car_autopilot::path_follower_example();

  ros::spin();

  return 0;
}
