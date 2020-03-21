#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <boost/any.hpp>
#include <basicmodutil_pkg/json.hpp>
#include <typeinfo>
#include <cctype>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

//...include as many messages your basic-module requires
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#ifndef __UTILBM__H
#define __UTILBM__H

namespace basicmodutil_pkg
{
	/*
	* \brief Method to obtain the data-type of a ROS message, in the format 'msg_package/msg_name'.
	* \param ros_msg ROS message instance.
	* \return String that contains the data-type of 'ros_msg'.
	*/
	std::string getRosType(const boost::any &ros_msg);
}

class BMTemplate
{
	private:
		//basic-module name
		std::string bm_name;

		//input parameters buffer
		std::map<std::string,boost::any> buffer;

		//data publishers
		std::map<std::string,ros::Publisher> data_pubs;

		//sync-topics
		std::string top_sync_pub;
		std::string top_sync_sub;

		//data-topics
		std::string top_data_twist;
		std::string top_data_float64;
		std::string top_data_point;
		std::string top_data_bool;

		//sync pub & sub
		ros::Publisher sync_pub;
		ros::Subscriber sync_sub;

		//data subs
		ros::Subscriber twist_sub;
		ros::Subscriber float64_sub;
		ros::Subscriber point_sub;
		ros::Subscriber bool_sub;

	public:

		/*
		 * \brief Constructor method.
		 * \param nh Pointer that is used to initiate the publishers and subscribers.
		 * \param bm_name Name of the basic module that is being instantiated.
		*/
		BMTemplate(ros::NodeHandle *nh, std::string bm_name);

		/*
		 * \brief Desctructor method.
		*/
		~BMTemplate()
		{
		}

	private:

		/*
		 * \brief This method handles the event of receiving input-params and invokes the requested basic module function.
		 * \param msg Pointer of the std_msgs/String message received.
		*/
		void sync_cb(const std_msgs::String::ConstPtr& msg);

		/*
		 * \brief
		 * \param param_names Vector of names of the output-params for the ROS messages in 'params'.
		 * \param params Vector of ROS messages.
		 * \return True is returned if every message in 'params' was successfully published, otherwise, false.
		*/
		bool returnOutParams(std::vector<std::string> param_names, std::vector<boost::any> params);

		//Data-topic callbacks
		void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
		void float64_cb(const std_msgs::Float64::ConstPtr& msg);
		void point_cb(const geometry_msgs::Point::ConstPtr& msg);
		void bool_cb(const std_msgs::Bool::ConstPtr& msg);

		//Basic module's functions
		void desplazar(const geometry_msgs::Twist &param1,const std_msgs::Float64 &param2);
		void posicion(const geometry_msgs::Point &param1);
};

#endif
