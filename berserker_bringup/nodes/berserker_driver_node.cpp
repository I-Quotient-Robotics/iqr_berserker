#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign.hpp>
#include "CRobot.h"


#define PI 3.141592653589793
#define USING_ARDUINO_ODOM

class robot_node
{
  public:
	robot_node();
	// ~robot_node();

	void cmdCallback(const geometry_msgs::Twist &msg);
	void odomPublish();
	void odomCount();

	// publish config
	bool publish_flag;
	ros::Time now_pub_time, last_pub_time;
	ros::Duration publish_period;

  protected:
	// config parameters
	std::string robot_port;
	int pub_rate;
	double robot_wheel_base;
	double robot_wheel_diameter;
	double robot_max_speed;
	double robot_min_speed;
	double robot_max_acc;
	double robot_min_acc;
	bool odom_tf_publish_flag;
	std::string odom_id;
	std::string child_id;
	int robot_encoder_lines;
	float robot_reduction_ratio;

	// data variables
	double robot_x;
	double robot_y;
	double robot_yaw;
	double robot_v;
	double robot_yaw_v;

	// publish config
	bool first_publish_flag;

	// sub config
	ros::Time last_odom_time;

	// Intermediate variables
	int now_left_encoder;
	int now_right_encoder;
	float now_imu_yaw;
	int last_left_encoder;
	int last_right_encoder;
	float last_imu_yaw;

  private:
	ros::NodeHandle m_nh;

	ros::Subscriber m_cmd_sub;
	ros::Publisher m_odom_pub;

	tf::TransformBroadcaster m_odom_broadcaster;
	geometry_msgs::TransformStamped m_odom_trans;
	nav_msgs::Odometry m_odom_msg;

	geometry_msgs::Twist m_cmdvel;

	CRobot *m_pRobot;
};

robot_node::robot_node()
	:m_nh("~")
{
	std::vector<double> pose_cov_diag(6, 1e-6);
	std::vector<double> twist_cov_diag(6, 1e-6);

	m_nh.param<std::string>("robot/port", robot_port, "/dev/ttyACM0");
	m_nh.param<double>("robot/wheel_base", robot_wheel_base, 0.60);
	m_nh.param<double>("robot/wheel_diameter", robot_wheel_diameter, 0.35);
	m_nh.param<double>("robot/max_speed", robot_max_speed, 2.0);
	m_nh.param<double>("robot/min_speed", robot_min_speed, -2.0);
	m_nh.param<double>("robot/max_acc", robot_max_acc, 0.5);
	m_nh.param<double>("robot/min_acc", robot_min_acc, -0.5);
	m_nh.param<int>("robot/encoder_lines", robot_encoder_lines, 1000);
	m_nh.param<float>("robot/reduction_ratio", robot_reduction_ratio, 10.0);

	m_nh.param<std::string>("odometry/odom_id", odom_id, "odom");
	m_nh.param<std::string>("odometry/child_id", child_id, "base_link");
	m_nh.param<int>("odometry/pub_rate", pub_rate, 30);
	m_nh.param<bool>("odometry/tf_publish_flag", odom_tf_publish_flag, true);



	if (m_nh.hasParam("odometry/pose_covariance_diagonal"))
	{
		XmlRpc::XmlRpcValue pose_cov_list;
		m_nh.getParam("odometry/pose_covariance_diagonal", pose_cov_list);
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

	if (m_nh.hasParam("odometry/twist_covariance_diagonal"))
	{
		XmlRpc::XmlRpcValue twist_cov_list;
		m_nh.getParam("odometry/twist_covariance_diagonal", twist_cov_list);
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

	char *buf = new char[strlen(robot_port.c_str()) + 1];
	strcpy(buf, robot_port.c_str());
	m_pRobot = new CRobot(buf);
	// m_pRobot = new CRobot(robot_port.c_ptr()); // c++11不支持此用法

	// 发布器的一些设置
	publish_period = ros::Duration(1.0 / pub_rate);
	publish_flag = true;
	first_publish_flag = true;
	now_pub_time = ros::Time::now();

	// 初始化订阅器和发布器
	m_cmd_sub = m_nh.subscribe("cmd_vel", 50, &robot_node::cmdCallback, this);
	m_odom_pub = m_nh.advertise<nav_msgs::Odometry>("odom", 50);

	// 初始化变量
	robot_x = 0.0;
	robot_y = 0.0;
	robot_yaw = 0.0;
	robot_v = 0.0;
	robot_yaw_v = 0.0;

	now_left_encoder = 0;
	now_right_encoder = 0;
	now_imu_yaw = 0.0;
	last_left_encoder = 0;
	last_right_encoder = 0;
	last_imu_yaw = 0.0;

	if (odom_tf_publish_flag)
	{
		m_odom_trans.header.frame_id = odom_id;
		m_odom_trans.child_frame_id = child_id;
		m_odom_trans.transform.translation.z = 0.0;
		m_odom_trans.transform.translation.x = robot_x;
		m_odom_trans.transform.translation.y = robot_y;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_yaw);
		m_odom_trans.transform.rotation = odom_quat;

		m_odom_broadcaster.sendTransform(m_odom_trans);
	}

	m_odom_msg.header.frame_id = odom_id;
	m_odom_msg.child_frame_id = child_id;
	m_odom_msg.pose.pose.position.z = 0.0;
	m_odom_msg.pose.covariance = boost::assign::list_of(pose_cov_diag[0])(0)(0)(0)(0)(0)(0)(pose_cov_diag[1])(0)(0)(0)(0)(0)(0)(pose_cov_diag[2])(0)(0)(0)(0)(0)(0)(pose_cov_diag[3])(0)(0)(0)(0)(0)(0)(pose_cov_diag[4])(0)(0)(0)(0)(0)(0)(pose_cov_diag[5]);
	m_odom_msg.twist.twist.linear.y = 0.0;
	m_odom_msg.twist.twist.linear.z = 0.0;
	m_odom_msg.twist.twist.angular.x = 0.0;
	m_odom_msg.twist.twist.angular.y = 0.0;
	m_odom_msg.twist.covariance = boost::assign::list_of(twist_cov_diag[0])(0)(0)(0)(0)(0)(0)(twist_cov_diag[1])(0)(0)(0)(0)(0)(0)(twist_cov_diag[2])(0)(0)(0)(0)(0)(0)(twist_cov_diag[3])(0)(0)(0)(0)(0)(0)(twist_cov_diag[4])(0)(0)(0)(0)(0)(0)(twist_cov_diag[5]);

	m_pRobot->start();
	m_pRobot->start_2();
}
void robot_node::cmdCallback(const geometry_msgs::Twist &msg)
{
	// TODO :速度限制，加速度限制
	ROS_INFO("[%f,%f]", msg.linear.x, msg.angular.z);
	m_pRobot->setRobotSpeed(msg.linear.x, msg.angular.z);
}

void robot_node::odomPublish()
{
	now_pub_time = ros::Time::now();

#ifdef USING_ARDUINO_ODOM

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_pRobot->readOdom(TH));

	// Transform message
	if (odom_tf_publish_flag)
	{
		m_odom_trans.header.stamp = now_pub_time;
		m_odom_trans.transform.translation.x = m_pRobot->readOdom(X);
		m_odom_trans.transform.translation.y = m_pRobot->readOdom(Y);
		m_odom_trans.transform.rotation = odom_quat;
		m_odom_broadcaster.sendTransform(m_odom_trans);
	}

	// Odometry message
	m_odom_msg.header.stamp = now_pub_time;
	m_odom_msg.pose.pose.position.x = m_pRobot->readOdom(X);
	m_odom_msg.pose.pose.position.y = m_pRobot->readOdom(Y);
	m_odom_msg.pose.pose.orientation = odom_quat;
	m_odom_msg.twist.twist.linear.x = m_pRobot->readOdom(V);
	m_odom_msg.twist.twist.angular.z = m_pRobot->readOdom(VTH);
	m_odom_pub.publish(m_odom_msg);

#else

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_yaw);

	// Transform message
	if (odom_tf_publish_flag)
	{
		m_odom_trans.header.stamp = now_pub_time;
		m_odom_trans.transform.translation.x = robot_x;
		m_odom_trans.transform.translation.y = robot_y;
		m_odom_trans.transform.rotation = odom_quat;
		m_odom_broadcaster.sendTransform(m_odom_trans);
	}

	// Odometry message
	m_odom_msg.header.stamp = now_pub_time;
	m_odom_msg.pose.pose.position.x = robot_x;
	m_odom_msg.pose.pose.position.y = robot_y;
	m_odom_msg.pose.pose.orientation = odom_quat;
	m_odom_msg.twist.twist.linear.x = robot_v;
	m_odom_msg.twist.twist.angular.z = robot_yaw_v;
	m_odom_pub.publish(m_odom_msg);
#endif
}

void robot_node::odomCount()
{
	// TODO: 里程计推算
	ros::Time now_time = ros::Time::now();

	now_left_encoder = 0;//m_pRobot->readMotorEncoder(LEFT_MOTOR);
	now_right_encoder = 0;//m_pRobot->readMotorEncoder(RIGHT_MOTOR);


	const double ds_left = (PI * robot_wheel_diameter) * (now_left_encoder - last_left_encoder) / (robot_encoder_lines);
	const double ds_right = (PI * robot_wheel_diameter) * (now_right_encoder - last_right_encoder) / (robot_encoder_lines);
	const double dimu_yaw = now_imu_yaw - last_imu_yaw;

	const double dt = (now_time - last_odom_time).toSec();
	const double ds = (ds_left + ds_right) / 2.0;
	const double dyaw = ((ds_left - ds_right) / robot_wheel_base);

	robot_v = ds / dt;
	robot_yaw_v = dyaw / dt;

	if (fabs(robot_yaw_v) > 1e-6) // 转弯啦
	{
		const double r = ds / dyaw;
		robot_x += r * (sin(robot_yaw + dyaw) - sin(robot_yaw));
		robot_y -= r * (cos(robot_yaw + dyaw) - cos(robot_yaw));
		robot_yaw += dyaw;
	}
	else // 直行
	{
		robot_x += ds * cos(robot_yaw + 0.5 * dyaw);
		robot_y += ds * sin(robot_yaw + 0.5 * dyaw);
		robot_yaw += dyaw;
	}
	last_left_encoder = now_left_encoder;
	last_right_encoder = now_right_encoder;
	last_imu_yaw = now_imu_yaw;
	last_odom_time = now_time;
	publish_flag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "berserker_driver");
	robot_node nd;
	ros::Rate r(30);

	while (ros::ok())
	{
		ros::spinOnce();
		//nd.odomCount();
		if (((nd.last_pub_time + nd.publish_period) < ros::Time::now()) && nd.publish_flag)
		{
			nd.odomPublish();
			nd.publish_flag = true;
			nd.last_pub_time = ros::Time::now();
		}
		r.sleep();
	}
	return 0;
}
