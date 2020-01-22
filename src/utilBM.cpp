#include <basicmodutil_pkg/utilBM.hpp>

using json = nlohmann::json;
using namespace std;

string getRosType(const boost::any &ros_msg)
{
	//Remove the extra-text at both ends
	const type_info &ti = ros_msg.type();
	string tmp(ti.name());
	if(tmp.length() <= 0) return string("");//check for empty string
	tmp = tmp.substr(1);
	int left_dig(0);
	for(int i = 0; i < tmp.length(); i++)
	{
		if(isdigit(tmp[i])) left_dig++;
		else break;
	}
	tmp = tmp.substr(left_dig);
	int idx = tmp.find("_ISaIvEEE");
	if(idx == -1) return string("");//check for not ros-msg
	tmp = tmp.substr(0,idx);

	//Find the largest substring in 'tmp' that is an existing
	//ROS package
	int pos(-1);
	for(int i = 0; i < tmp.length(); i++)
	{
		if(!isdigit(tmp[i])) continue;

		string pack = tmp.substr(0,i);
		string path = ros::package::command("-q find "+pack);
		if(path.length() > 0)
		{
			pos = i;
		}
	}

	//Just in case a non-considered type_info string
	if(pos == -1) return string("");

	//Get the package name
	string pack = tmp.substr(0,pos);

	//Get the message name
	tmp = tmp.substr(pos);
	pos = -1;
	for(int i = 0; i < tmp.length(); i++)
	{
		if(!isdigit(tmp[i]))
		{
			pos = i;
			break;
		}
	}
	string msg = tmp.substr(pos);

	//Build the message type
	string ros_type_msg = pack + "/" + msg;

	return ros_type_msg;
}

BMTemplate::BMTemplate(ros::NodeHandle *nh, string bm_name)
{
	//basic-module name
	this->bm_name = bm_name;

	//sync-topics
	top_sync_pub = "/"+bm_name+"/BM_output";
	top_sync_sub = "/"+bm_name+"/BM_input";

	//data-topics
	top_data_string = "/"+bm_name+"/string";
	top_data_bool = "/"+bm_name+"/bool";
	// ... and as many as the basic module requires

	//sync pub & sub
	sync_pub = nh->advertise<std_msgs::String>(top_sync_pub,1);
	sync_sub = nh->subscribe<std_msgs::String>(top_sync_sub,1,&BMTemplate::sync_cb,this);

	//data pubs & subs
	ros::Publisher string_pub = nh->advertise<std_msgs::String>(top_data_string,1);
	string_sub = nh->subscribe<std_msgs::String>(top_data_string,1,&BMTemplate::string_cb,this);
	ros::Publisher bool_pub = nh->advertise<std_msgs::Bool>(top_data_bool,1);
	bool_sub = nh->subscribe<std_msgs::Bool>(top_data_bool,1,&BMTemplate::bool_cb,this);
	// ... and as many as the basic module requires

	//Store data publishers in the std::map 'data_pubs'
	// key = data-type, value = ROS-publisher
	data_pubs[getRosType(std_msgs::String())] = string_pub;
	data_pubs[getRosType(std_msgs::Bool())] = bool_pub;
	// ... and as many as the basic module requires

	//Create an attribute in the buffer for each data-topic
	// key = data-topic, value = ROS-message object
	buffer[top_data_string] = std_msgs::String();
	buffer[top_data_bool] = std_msgs::Bool();
	// ... and as many as the basic module requires

	//DISPLAY IN CONSOLE OF THE BASIC-MODULE
	ROS_INFO("BASIC-MODULE");
	ROS_INFO("\t%s",bm_name.c_str());
	ROS_INFO("SYNC-TOPICS");
	ROS_INFO("\t%s",top_sync_pub.c_str());
	ROS_INFO("\t%s",top_sync_sub.c_str());
	ROS_INFO("DATA-TOPICS");
	ROS_INFO("\t%s",top_data_string.c_str());
	ROS_INFO("\t%s",top_data_bool.c_str());
	ROS_INFO("######################################");
}

void BMTemplate::sync_cb(const std_msgs::String::ConstPtr& msg)
{
	try
	{
		//Parse the JSON string into an object
		string js(msg->data);
		json jj = json::parse(js);

		//Get the requested function and its input params
		string fun = jj["function"];
		vector<string> inparam_topics = jj["inparam_topics"];
		vector<boost::any> inparams;
		for(unsigned int i = 0; i < inparam_topics.size(); i++)
		{
			inparams.push_back(buffer[inparam_topics[i]]);
		}

		//DISPLAY IN CONSOLE
		ROS_INFO("SYNC_CB");
		ROS_INFO("FUNCTION");
		ROS_INFO("\t%s",fun.c_str());
		ROS_INFO("INPARAM-TOPICS");
		for(unsigned int i = 0; i < inparam_topics.size(); i++)
			ROS_INFO("\t%s",inparam_topics[i].c_str());
		ROS_INFO("--------------------------------");

		/*
			NOTE:
			At this point, 'fun' holds the name of the requested function and 'inparams'
			is a vector that contains the function's input parameters ROS-message objects,
			in the order in which they were defined in the basic module's description.
		*/

		//Invoke the requested function
		if(fun == "functionA")
		{
			//invoke 'functionA'
			auto param1 = boost::any_cast<std_msgs::String>(buffer[inparam_topics[0]]);
			auto param2 = boost::any_cast<std_msgs::Bool>(buffer[inparam_topics[1]]);
			functionA(param1,param2);
		}
		else if(fun == "functionB")
		{
			//invoke 'functionB'
			auto param1 = boost::any_cast<std_msgs::Bool>(buffer[inparam_topics[0]]);
			auto param2 = boost::any_cast<std_msgs::String>(buffer[inparam_topics[1]]);
			functionB(param1,param2);
		}
		//Add a case for each function in the basic module

	}
	catch(std::exception& e)
	{
		ROS_WARN("%s received a non-JSON msg. in the SYNC callback.",this->bm_name.c_str());
		return;
	}
}

bool BMTemplate::returnOutParams(vector<string> param_names, vector<boost::any> params)
{
	if(param_names.size() != params.size()) return false;

	//Get the output-param names
	json jj;
	jj["outparam_names"] = param_names;

	//Publish the output-params in their respective publisher
	vector<string> outparam_topics;
	for(unsigned int i = 0; i < params.size(); i++)
	{
		string ros_type = getRosType(params[i]);

		//Cast the i-th param to its data-type
		if(ros_type  == "std_msgs/String")
			data_pubs[ros_type].publish(boost::any_cast<std_msgs::String>(params[i]));
		else if(ros_type  == "std_msgs/Bool")
			data_pubs[ros_type].publish(boost::any_cast<std_msgs::Bool>(params[i]));
		//... and as many cases for the data-types the basic-module has


		//Get the topic in which 'params[i]' is published
		outparam_topics.push_back(data_pubs[ros_type].getTopic());
	}

	//Make a small delay
	ros::Duration(1.0).sleep();

	//Publish in the sync topic the name & topics of the output parameters
	jj["outparam_topics"] = outparam_topics;
	string js = jj.dump();
	std_msgs::String msg;
	msg.data = js;
	sync_pub.publish(msg);
}

void BMTemplate::string_cb(const std_msgs::String::ConstPtr& msg)
{
	//Store the received data in the buffer
	try
	{
		buffer[top_data_string] = *msg;
	}
	catch(std::exception& e)
	{
		return;
	}
}

void BMTemplate::bool_cb(const std_msgs::Bool::ConstPtr& msg)
{
	//Store the received data in the buffer
	try
	{
		buffer[top_data_bool] = *msg;
	}
	catch(std::exception& e)
	{
		return;
	}
}

void BMTemplate::functionA(const std_msgs::String &param1,const std_msgs::Bool &param2)
{
	//DISPLAY IN CONSOLE
	ROS_INFO("EXECUTING FUNCTION");
	ROS_INFO("\tfunctionA");
	cout << ">> Processing: " << param1.data << endl;
	cout << ">> Processing: " << bool(param2.data) << endl;

	vector<string> tmp_n;
	tmp_n.push_back(string("status"));
	vector<boost::any> tmp_p;
	std_msgs::String outp;
	outp.data = "Successful";
	tmp_p.push_back(outp);

	//Return the function's output-parameters
	returnOutParams(tmp_n,tmp_p);
	ROS_INFO("DONE");
	ROS_INFO("--------------------------------");
}

void BMTemplate::functionB(const std_msgs::Bool &param1,const std_msgs::String &param2)
{
	//DISPLAY IN CONSOLE
	ROS_INFO("EXECUTING FUNCTION");
	ROS_INFO("\tfunctionB");
	cout << ">> Processing: " << bool(param1.data) << endl;
	cout << ">> Processing: " << param2.data << endl;

	vector<string> tmp_n;
	tmp_n.push_back(string("flag"));
	vector<boost::any> tmp_p;
	std_msgs::Bool outp;
	outp.data = true;
	tmp_p.push_back(outp);

	//Return the function's output-parameters
	returnOutParams(tmp_n,tmp_p);
	ROS_INFO("DONE");
	ROS_INFO("--------------------------------");
}




