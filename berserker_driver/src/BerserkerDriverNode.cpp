/**********************************************************************************
** MIT License
** 
** Copyright (c) 2019 I-Quotient-Robotics https://github.com/I-Quotient-Robotics
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** 
** The above copyright notice and this permission notice shall be included in all
** copies or substantial portions of the Software.
** 
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
** SOFTWARE.
*********************************************************************************/
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/assign.hpp>
#include "BerserkerDriver.h"

using namespace std;

class BerserkerDriverNode
{
public:
  BerserkerDriverNode();
  ~BerserkerDriverNode();
  void pubJointState();
  void pubOdom();

protected:
  void callBack(const geometry_msgs::Twist &msg);
  //ros params
  std::string _portName;
  std::string _leftWheelJointName;
  std::string _rightWheelJointName;

  std::string _odom_id;
  std::string _child_id;
  boost::assign_detail::generic_list<double> _pose_covariance;
  boost::assign_detail::generic_list<double> _twist_covariance;

private:
  IQR::BerserkerDriver *_bd;

  ros::NodeHandle _nh;
  ros::Subscriber _cmdSub;
  ros::Publisher _odomPub;
  ros::Publisher _jointPub;
  sensor_msgs::JointState _js;
  tf::TransformBroadcaster odom_broadcaster;
};

BerserkerDriverNode::BerserkerDriverNode()
    : _nh("~")
{
  //init params
  _nh.param<std::string>("robot_port", _portName, "/dev/ttyACM0");
  _nh.param<std::string>("robot_left_wheel_joint_name", _leftWheelJointName, "FL_wheel_joint");
  _nh.param<std::string>("robot_right_wheel_joint_name", _rightWheelJointName, "FR_wheel_joint");
  _nh.param<std::string>("odom_id", _odom_id, "odom");
  _nh.param<std::string>("odom_child_id", _child_id, "base_footprint");

  std::vector<double> pose_cov_diag(6, 1e-6);
  std::vector<double> twist_cov_diag(6, 1e-6);

  if (_nh.hasParam("odom_pose_covariance_diagonal"))
  {
    XmlRpc::XmlRpcValue pose_cov_list;
    _nh.getParam("odom_pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
    {
      // Read as string to handle no decimals and scientific notation
      std::ostringstream ostr;
      ostr << pose_cov_list[i];
      std::istringstream istr(ostr.str());
      istr >> pose_cov_diag[i];
    }
  }
  else
  {
    ROS_WARN("Pose covariance diagonals not specified for odometry integration. Defaulting to 1e-6.");
  }

  if (_nh.hasParam("odom_twist_covariance_diagonal"))
  {
    XmlRpc::XmlRpcValue twist_cov_list;
    _nh.getParam("odom_twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
    {
      // Read as string to handle no decimals and scientific notation
      std::ostringstream ostr;
      ostr << twist_cov_list[i];
      std::istringstream istr(ostr.str());
      istr >> twist_cov_diag[i];
    }
  }
  else
  {
    ROS_WARN("Twist covariance diagonals not specified for odometry integration. Defaulting to 1e-6.");
  }

  _pose_covariance = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)(0)(pose_cov_diag[1])(0)(0)(0)(0)(0)(0)(pose_cov_diag[2])(0)(0)(0)(0)(0)(0)(pose_cov_diag[3])(0)(0)(0)(0)(0)(0)(pose_cov_diag[4])(0)(0)(0)(0)(0)(0)(pose_cov_diag[5]);
  _twist_covariance = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)(0)(twist_cov_diag[1])(0)(0)(0)(0)(0)(0)(twist_cov_diag[2])(0)(0)(0)(0)(0)(0)(twist_cov_diag[3])(0)(0)(0)(0)(0)(0)(twist_cov_diag[4])(0)(0)(0)(0)(0)(0)(twist_cov_diag[5]);

  //joint_states
  _js.name.resize(2);
  _js.position.resize(2);
  _js.velocity.resize(2);
  _js.effort.resize(2);
  _js.name[0] = _leftWheelJointName;
  _js.name[1] = _rightWheelJointName;

  //set publisher
  _odomPub = _nh.advertise<nav_msgs::Odometry>("odom", 50);
  _jointPub = _nh.advertise<sensor_msgs::JointState>("joint_states", 50);

  //set subscriber
  _cmdSub = _nh.subscribe("cmd_vel", 50, &BerserkerDriverNode::callBack, this);

  cout << "robot port name:" << _portName << endl;
  _bd = new IQR::BerserkerDriver(_portName);
}

BerserkerDriverNode::~BerserkerDriverNode()
{
  delete _bd;
}

void BerserkerDriverNode::pubJointState()
{
  float l_w_s = 0.0;
  float r_w_s = 0.0;

  _bd->getWheelSpeed(l_w_s, r_w_s);

  _js.header.stamp = ros::Time::now();
  _js.position[0] = l_w_s;
  _js.position[1] = r_w_s;
  _jointPub.publish(_js);
}

void BerserkerDriverNode::pubOdom()
{
  static double x = 0.0;
  static double y = 0.0;
  static double th = 0.0;
  static ros::Time now_time = ros::Time::now();
  static ros::Time last_time = ros::Time::now();

  now_time = ros::Time::now();

  int32_t nLE, nRE;
  float nLS, nRS;
  _bd->getEncoder(nLE, nRE);
  _bd->getWheelSpeed(nLS, nRS);

  ROS_INFO("encoder:[%d,%d], speed:[%f,%f]", nLE, nRE, nLS, nRS);

  double vx = (nLS + nRS) / 2.0;
  double vth = (nRS - nLS) / 0.60;

  //compute odometry in a typical way given the velocities of the robot
  double dt = (now_time - last_time).toSec();
  double delta_x = (vx * cos(th)) * dt;
  double delta_y = (vx * sin(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now_time;
  odom_trans.header.frame_id = _odom_id;
  odom_trans.child_frame_id = _child_id;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = now_time;
  odom.header.frame_id = _odom_id;

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = _pose_covariance;
  //set the velocity
  odom.child_frame_id = _child_id;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vth;
  odom.twist.covariance = _twist_covariance;
  
  _odomPub.publish(odom);

  last_time = now_time;
}

void BerserkerDriverNode::callBack(const geometry_msgs::Twist &msg)
{
  ROS_INFO("[%f,%f]", msg.linear.x, msg.angular.z);
  _bd->setSpeed(msg.linear.x, msg.angular.z);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "BerserkerDriverNode");

  BerserkerDriverNode nd;
  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
    nd.pubJointState();
    nd.pubOdom();
    r.sleep();
  }
  return 0;
}
