// Name: Jerel Nielsen
// Date: 15 June 2017
// Desc: Container for common ROS-related functions.

#include "common_ros/common_ros.h"

namespace common
{


void getQuaternionFromTF(std::string frame1, std::string frame2, common::Quaternion& q)
{
  // intantiate a ROS tf listener
  tf::TransformListener tf_listener;

  // lookup the transform (from tf tree) from frame1 to frame2
  tf::StampedTransform transform;
  try 
  {
    tf_listener.waitForTransform(frame1, frame2, ros::Time(0), ros::Duration(10.0) );
    tf_listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
  } 
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to my quaternion class object
  tf::Quaternion _q = transform.getRotation();
  q.w = _q.w();
  q.x = _q.x();
  q.y = _q.y();
  q.z = _q.z();
}


} // namespace common


