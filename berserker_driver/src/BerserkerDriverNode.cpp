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

private:
  void callBack(const geometry_msgs::Twist &msg);
  
  //ros
  ros::NodeHandle nh_;
  ros::Subscriber cmdSub_;
  ros::Publisher odomPub_;
  ros::Publisher jointPub_;
  sensor_msgs::JointState js_;
  tf::TransformBroadcaster odom_broadcaster_;
  //ros params
  std::string portName_;
  std::string odomName_;
  std::string childName_;
  bool odom_tf_publish_flag_;
  std::string leftWheelJointName_;
  std::string rightWheelJointName_;
  boost::assign_detail::generic_list<double> pose_covariance_;
  boost::assign_detail::generic_list<double> twist_covariance_;
  //robot driver
  IQR::BerserkerDriver *bd_;
};

BerserkerDriverNode::BerserkerDriverNode()
    : nh_("~")
{
  //init params
  nh_.param<std::string>("robot_port", portName_, "/dev/ttyACM0");
  nh_.param<std::string>("robot_left_wheel_joint_name", leftWheelJointName_, "FL_wheel_joint");
  nh_.param<std::string>("robot_right_wheel_joint_name", rightWheelJointName_, "FR_wheel_joint");
  nh_.param<std::string>("odom_id", odomName_, "odom");
  nh_.param<std::string>("odom_child_id", childName_, "base_footprint");
  nh_.param<bool>("odom_tf_publish_flag", odom_tf_publish_flag_, true);

  std::vector<double> pose_cov_diag(6, 1e-6);
  std::vector<double> twist_cov_diag(6, 1e-6);

  if (nh_.hasParam("odom_pose_covariance_diagonal"))
  {
    XmlRpc::XmlRpcValue pose_cov_list;
    nh_.getParam("odom_pose_covariance_diagonal", pose_cov_list);
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

  if (nh_.hasParam("odom_twist_covariance_diagonal"))
  {
    XmlRpc::XmlRpcValue twist_cov_list;
    nh_.getParam("odom_twist_covariance_diagonal", twist_cov_list);
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

  pose_covariance_ = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)(0)(pose_cov_diag[1])(0)(0)(0)(0)(0)(0)(pose_cov_diag[2])(0)(0)(0)(0)(0)(0)(pose_cov_diag[3])(0)(0)(0)(0)(0)(0)(pose_cov_diag[4])(0)(0)(0)(0)(0)(0)(pose_cov_diag[5]);
  twist_covariance_ = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)(0)(twist_cov_diag[1])(0)(0)(0)(0)(0)(0)(twist_cov_diag[2])(0)(0)(0)(0)(0)(0)(twist_cov_diag[3])(0)(0)(0)(0)(0)(0)(twist_cov_diag[4])(0)(0)(0)(0)(0)(0)(twist_cov_diag[5]);

  //joint_states
  js_.name.resize(2);
  js_.position.resize(2);
  js_.velocity.resize(2);
  js_.effort.resize(2);
  js_.name[0] = leftWheelJointName_;
  js_.name[1] = rightWheelJointName_;

  //set publisher
  odomPub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  jointPub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  //set subscriber
  cmdSub_ = nh_.subscribe("cmd_vel", 1, &BerserkerDriverNode::callBack, this);

  ROS_INFO("Berserker port name: %s", portName_.c_str());
  bd_ = new IQR::BerserkerDriver(portName_);
}

BerserkerDriverNode::~BerserkerDriverNode()
{
  delete bd_;
}

void BerserkerDriverNode::pubJointState()
{
  float l_w_s = 0.0;
  float r_w_s = 0.0;

  bd_->getWheelSpeed(l_w_s, r_w_s);

  js_.header.stamp = ros::Time::now();
  js_.velocity[0] = l_w_s;
  js_.velocity[1] = r_w_s;
  jointPub_.publish(js_);
}

void BerserkerDriverNode::pubOdom()
{
  static int32_t oLE = 0;
  static int32_t oRE = 0;
  static double x = 0.0;
  static double y = 0.0;
  static double th = 0.0;
  static ros::Time now_time;
  static ros::Time last_time;

  now_time = ros::Time::now();

  int32_t nLE, nRE;
  float nLS, nRS;
  bd_->getEncoder(nLE, nRE);
  bd_->getWheelSpeed(nLS, nRS);

  //ROS_INFO("encoder:[%d,%d], speed:[%f,%f]", nLE, nRE, nLS, nRS);
  //ROS_INFO("X:%f, Y:%F, TH:%f", x, y, th);

  //calculate dt
  if (last_time.isZero())
  {
    oLE = nLE;
    oRE = nRE;
    last_time = now_time;
    return;
  }

  double dt = (now_time - last_time).toSec();
  if (dt == 0)
    return;
  last_time = now_time;

  //calculate sLS sRS
  double dLD = (nLE - oLE) * 1.09955741e-4;
  double dRD = (nRE - oRE) * 1.09955741e-4;
  double sLS = dLD / dt;
  double sRS = dRD / dt;
  oLE = nLE;
  oRE = nRE;

  js_.header.stamp = ros::Time::now();
  js_.position[0] = nLE * 6.2831852e-4;
  js_.position[1] = nRE * 6.2831852e-4;
  js_.velocity[0] = sLS;
  js_.velocity[1] = sRS;
  jointPub_.publish(js_);

  //calculate vx vth
  //double vx =  (nRS + nLS) / 2.0;
  //double vth = (nRS - nLS) / 0.60;
  double vx = (sRS + sLS) / 2.0;
  double vth = (sRS - sLS) / 0.60;

  //calculate dx dy dth
  double delta_x = (vx * cos(th)) * dt;
  double delta_y = (vx * sin(th)) * dt;
  double delta_th = vth * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;

  //calculate x y th
  // double dS = (dLD + dRD)/2.0;
  // double dTH = (dRD - dLD)/0.6;
  // if (fabs(dTH) > 1e-4)
  // {
  // 	double r = fabs(dS / dTH);
  // 	x -= r * (sin(th + dTH) - sin(th));
  // 	y += r * (cos(th + dTH) - cos(th));
  // 	th += dTH;
  // }
  // else
  // {
  // 	x += dS * cos(th);
  // 	y += dS * sin(th);
  // 	th += dTH;
  // }

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = now_time;
  odom_trans.header.frame_id = odomName_;
  odom_trans.child_frame_id = childName_;

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  if (odom_tf_publish_flag_)
  {
    odom_broadcaster_.sendTransform(odom_trans);
  }

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = now_time;
  odom.header.frame_id = odomName_;

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.pose.covariance = pose_covariance_;
  //set the velocity
  odom.child_frame_id = childName_;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = vth;
  odom.twist.covariance = twist_covariance_;

  odomPub_.publish(odom);
}

void BerserkerDriverNode::callBack(const geometry_msgs::Twist &msg)
{
  //ROS_INFO("[%f,%f]", msg.linear.x, msg.angular.z);
  bd_->setSpeed(msg.linear.x, msg.angular.z);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "BerserkerDriverNode");

  BerserkerDriverNode nd;
  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
    //nd.pubJointState();
    nd.pubOdom();
    r.sleep();
  }
  return 0;
}
