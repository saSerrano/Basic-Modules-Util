#include <string>
#include <ros/ros.h>
#include <basicmodutil_pkg/utilBM.hpp>

using namespace std;

int main(int argc, char **argv)
{
	string basic_module_name("DEMO_BM");

	ros::init(argc,argv,basic_module_name);

	ros::NodeHandle nh_("~");

	BMTemplate bmt(&nh_, basic_module_name);

	ros::spin();

	return 0;
}

