#ifndef PATH_FOLLOWER_BASE_H
#define PATH_FOLLOWER_BASE_H

#include <ros/ros.h>
#include <car_autopilot/State.h>
#include <car_autopilot/Controller_Commands.h>
#include <dynamic_reconfigure/server.h>
//#include <car_autopilot/FollowerConfig.h>
#include <car_autopilot/Current_Path.h>


namespace car_autopilot
{

enum class path_type
{
  Orbit,
  Line
};

class path_follower_base
{
public:
  path_follower_base();
  float spin();

protected:

  struct input_s
  {
    enum path_type p_type;
    float u_d;
    float r_path[3];
    float q_path[3];
    float c_orbit[3];
    float rho_orbit;
    int lam_orbit;
    float pn;               /** position north */
    float pe;               /** position east */
    float u;               /** speed */
    float psi;              /** heading angle */
  };

  struct output_s
  {
    double u_c;             /** commanded speed (m/s) */
    double psi_c;            /** commanded heading (rad) */
  };

  struct params_s
  {
    double psi_infty;
    double k_path;
    double k_orbit;
  };

  virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;
  ros::Subscriber current_path_sub_;

  ros::Publisher controller_commands_pub_;

  double update_rate_ = 100.0;
  ros::Timer update_timer_;

  car_autopilot::Controller_Commands controller_commands_;
  struct params_s  params_;            /**< params */
  struct input_s input_;

  void vehicle_state_callback(const car_autopilot::StateConstPtr &msg);
  bool state_init_;
  void current_path_callback(const car_autopilot::Current_PathConstPtr &msg);
  bool current_path_init_;

//  dynamic_reconfigure::Server<car_autopilot::FollowerConfig> server_;
//  dynamic_reconfigure::Server<car_autopilot::FollowerConfig>::CallbackType func_;
//  void reconfigure_callback(car_autopilot::FollowerConfig &config, uint32_t level);

  void update(const ros::TimerEvent &);
};

} // end namespace

#endif // PATH_FOLLOWER_BASE_H
