/**
 * @file path_manager_base.h
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 * adapted by Judd Mehr and Brian Russel for RosPlane software
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

#include <ros/ros.h>
#include <car_autopilot/State.h>
#include <car_autopilot/Current_Path.h>
#include <car_autopilot/Waypoint.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>
#include <Eigen/Eigen>

namespace car_autopilot
{
class path_manager_base
{
public:
  path_manager_base();

protected:

  struct waypoint_s
  {
    float w[2];
    float u_d;
  };

  std::vector<waypoint_s> waypoints_;
  int num_waypoints_;
  int idx_a_;                 /** index to the waypoint that was most recently achieved */

  struct input_s
  {
    float pn;               /** position north */
    float pe;               /** position east */
    float psi;              /** course angle */
  };

  struct output_s
  {
    bool  flag;             /** Inicates strait line or orbital path (true is line, false is orbit) */
    float u_d;             /** Desired airspeed (m/s) */
    float r[2];             /** Vector to origin of straight line path (m) */
    float q[2];             /** Unit vector, desired direction of travel for line path */
    float c[2];             /** Center of orbital path (m) */
    float rho;              /** Radius of orbital path (m) */
    int8_t lambda;          /** Direction of orbital path (cw is 1, ccw is -1) */
  };

  struct params_s
  {
    double R_min;
  };

  virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber vehicle_state_sub_;     /**< vehicle state subscription */
  ros::Subscriber new_waypoint_sub_;      /**< new waypoint subscription */
  ros::Subscriber print_list_sub_;
  ros::Publisher  current_path_pub_;      /**< controller commands publication */

  struct params_s params_;

  car_autopilot::State vehicle_state_;     /**< vehicle state */

  double update_rate_;
  ros::Timer update_timer_;

  void vehicle_state_callback(const car_autopilot::StateConstPtr &msg);
  bool state_init_;
  void new_waypoint_callback(const car_autopilot::Waypoint &msg);
  void print_waypoints_callback(const std_msgs::Bool msg);
  void current_path_publish(const ros::TimerEvent &);
};
} //end namespace
#endif // PATH_MANAGER_BASE_H
