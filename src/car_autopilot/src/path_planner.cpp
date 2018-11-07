#include <ros/ros.h>
#include <car_autopilot/Waypoint.h>

#define num_waypoints 4

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_path_planner");

  ros::NodeHandle nh_;
  ros::NodeHandle nh_ns_("~");
  ros::Publisher waypointPublisher = nh_.advertise<car_autopilot::Waypoint>("waypoint_path", 10);

//  float u = 1.5;
//  float pn1 = -230.0;
//  float pe1 = -100.0;
//  float wps[3*num_waypoints] =
//  {
//    pn1 - 10, pe1, u,
//    pn1, pe1, u,
//    pn1, pe1 + 10, u,
//    pn1 - 10, pe1 + 10, u,
//  };
  std::vector<double> wps;
  nh_ns_.getParam("waypoint_list",wps);

//  for (int i(0); i < num_waypoints; i++)
  for (int i(0); i < wps.size()/3; i++)
  {
    ros::Duration(0.5).sleep();

    car_autopilot::Waypoint new_waypoint;

    new_waypoint.w[0] = wps[i*3 + 0];
    new_waypoint.w[1] = wps[i*3 + 1];
    new_waypoint.u_d = wps[i*3 + 2];

    if (i == 0)
      new_waypoint.set_current = true;
    else
      new_waypoint.set_current = false;
    new_waypoint.clear_wp_list = false;

    waypointPublisher.publish(new_waypoint);
  }
  ros::Duration(1.5).sleep();

  return 0;
}
