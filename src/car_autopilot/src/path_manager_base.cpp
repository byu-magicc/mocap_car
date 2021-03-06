#include "path_manager_base.h"
#include "path_manager_example.h"

namespace car_autopilot
{

path_manager_base::path_manager_base():
  nh_(ros::NodeHandle()), /** nh_ stuff added here */
  nh_private_(ros::NodeHandle("~"))
{
  nh_private_.param<double>("R_min", params_.R_min, 25.0);
  nh_private_.param<double>("update_rate", update_rate_, 10.0);

  vehicle_state_sub_ = nh_.subscribe("state", 10, &path_manager_base::vehicle_state_callback, this);
  new_waypoint_sub_ = nh_.subscribe("waypoint_path", 10, &path_manager_base::new_waypoint_callback, this);
  print_list_sub_ = nh_.subscribe("print_waypoint_path", 10, &path_manager_base::print_waypoints_callback, this);
  current_path_pub_ = nh_.advertise<car_autopilot::Current_Path>("current_path", 10);

  update_timer_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &path_manager_base::current_path_publish, this);

  num_waypoints_ = 0;

  state_init_ = false;
}

void path_manager_base::print_waypoints_callback(const std_msgs::Bool msg)
{
    for (int i(0);i<waypoints_.size();i++)
    {
        if(i == idx_a_)
            ROS_INFO("n:%f, e:%f <-- previousely achieved", waypoints_[i].w[0], waypoints_[i].w[1]);
        else
            ROS_INFO("n:%f, e:%f", waypoints_[i].w[0], waypoints_[i].w[1]);
    }
    ROS_INFO("\n");
}

void path_manager_base::vehicle_state_callback(const car_autopilot::StateConstPtr &msg)
{
  vehicle_state_ = *msg;

  state_init_ = true;
}

void path_manager_base::new_waypoint_callback(const car_autopilot::Waypoint &msg)
{
  if (msg.clear_wp_list == true)
  {
    waypoints_.clear();
    num_waypoints_ = 0;
    idx_a_ = 0;
    return;
  }
//  if (msg.set_current || num_waypoints_ == 0)
//  {
//    waypoint_s currentwp;
//    currentwp.w[0] = vehicle_state_.p_north;
//    currentwp.w[1] = vehicle_state_.p_east;
//    currentwp.u_d = msg.u_d;
//    waypoints_.clear();
//    waypoints_.push_back(currentwp);
//    num_waypoints_ = 1;
//    idx_a_ = 0;
//  }
  waypoint_s nextwp;
  nextwp.w[0]         = msg.w[0];
  nextwp.w[1]         = msg.w[1];
  nextwp.u_d = msg.u_d;
  waypoints_.push_back(nextwp);
  num_waypoints_++;
}

void path_manager_base::current_path_publish(const ros::TimerEvent &)
{

  struct input_s input;
  input.pn = vehicle_state_.p_north;               /** position north */
  input.pe = vehicle_state_.p_east;               /** position east */
  input.psi = vehicle_state_.psi;

  struct output_s output;

  if (state_init_ == true)
  {
    manage(params_, input, output);
  }

  car_autopilot::Current_Path current_path;

  if (output.flag)
    current_path.path_type = current_path.LINE_PATH;
  else
    current_path.path_type = current_path.ORBIT_PATH;
  current_path.u_d = output.u_d;
  for (int i = 0; i < 2; i++)
  {
    current_path.r[i] = output.r[i];
    current_path.q[i] = output.q[i];
    current_path.c[i] = output.c[i];
  }
  current_path.rho = output.rho;
  current_path.lambda = output.lambda;

  current_path_pub_.publish(current_path);
}

} //end namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_manager");
  car_autopilot::path_manager_base *est = new car_autopilot::path_manager_example();

  ros::spin();

  return 0;
}
