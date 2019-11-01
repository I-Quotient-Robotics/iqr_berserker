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
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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

private:
  IQR::BerserkerDriver *_bd;
  std::string _portName;
  std::string _leftWheelJointName;
  std::string _rightWheelJointName;
  sensor_msgs::JointState _js;
  ros::NodeHandle _nh;
  ros::Subscriber _cmdSub;
  ros::Publisher _jointPub;
  ros::Publisher _odomPub;
};

BerserkerDriverNode::BerserkerDriverNode()
    : _nh("~")
{
  //init params
  _nh.param<std::string>("port_name", _portName, "/dev/ttyACM0");
  _nh.param<std::string>("left_wheel_joint_name", _leftWheelJointName, "FL_wheel_joint");
  _nh.param<std::string>("right_wheel_joint_name", _rightWheelJointName, "FR_wheel_joint");

  //joint_states
  _js.name.resize(2);
  _js.position.resize(2);
  _js.velocity.resize(2);
  _js.effort.resize(2);
  _js.name[0] = _leftWheelJointName;
  _js.name[1] = _rightWheelJointName;

  //set publisher
  _jointPub = _nh.advertise<sensor_msgs::JointState>("/joint_states", 50);
  _odomPub = _nh.advertise<nav_msgs::Odometry>("odom", 50);

  //set subscriber
  _cmdSub = _nh.subscribe("cmd_vel", 50, &BerserkerDriverNode::callBack, this);

  cout << "robot port name:" << _portName << endl;
  _bd = new IQR::BerserkerDriver(_portName);
  sleep(2);
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
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  //odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0;
  _odomPub.publish(odom);
}

void BerserkerDriverNode::callBack(const  geometry_msgs::Twist &msg)
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
    r.sleep();
  }
  return 0;
}
