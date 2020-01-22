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
#include <std_msgs/Bool.h>

#ifndef __UTILBM__H
#define __UTILBM__H

/*
* \brief Method to obtain the data-type of a ROS message, in the format 'msg_package/msg_name'.
* \param ros_msg ROS message instance.
* \return String that contains the data-type of 'ros_msg'.
*/
std::string getRosType(const boost::any &ros_msg);

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
		std::string top_data_string;
		std::string top_data_bool;
		// ... and as many as the basic module requires

		//sync pub & sub
		ros::Publisher sync_pub;
		ros::Subscriber sync_sub;

		//data subs
		ros::Subscriber string_sub;
		ros::Subscriber bool_sub;
		// ... and as many as the basic module requires

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

		/*
		 * \brief Callback method to receive input-params through the 'std_msgs/String' data-topic.
		 * \param msg Pointer of the std_msgs/String message received.
		*/
		void string_cb(const std_msgs::String::ConstPtr& msg);

		/*
		 * \brief Callback method to receive input-params through the 'std_msgs/Bool' data-topic.
		 * \param msg Pointer of the std_msgs/Bool message received.
		*/
		void bool_cb(const std_msgs::Bool::ConstPtr& msg);

		/*
		 * \brief Example of a function for this basic-module template
		 * \param param1 First input-param
		 * \param param2 Second input-param
		*/
		void functionA(const std_msgs::String &param1,const std_msgs::Bool &param2);

		/*
		 * \brief Example of a function for this basic-module template
		 * \param param1 First input-param
		 * \param param2 Second input-param
		*/
		void functionB(const std_msgs::Bool &param1,const std_msgs::String &param2);
};

#endif
