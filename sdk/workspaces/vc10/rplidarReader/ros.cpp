#include "stdafx.h"

// %Tag(FULLTEXT)%
// %Tag(ROS_HEADER)%

#include <memory>

// %EndTag(MSG_HEADER)%

#include <sstream>

#include "/usr/include/rplidar/rosstuff.h"

bool ros::console::g_initialized = false;




ROS_INIT


extern "C"
{
	DLL_EXPORT void InitROSLidarPublisher(const std::string & pubname, const std::string & topic,int qSize, int argc, char **argv) {


	}

	DLL_EXPORT void PublishScan() {



		
	}

	
}


int main_(int argc, char **argv)
{

	ros::init(argc, argv, "talker");

	ros::NodeHandle nodeHandle;



	ros::Publisher thePublisher = nodeHandle.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);
	

	int count = 0;
	while (ros::ok())
	{

		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
	
		thePublisher.publish(msg);



		ros::spinOnce();


	
		loop_rate.sleep();
	
		++count;
	}


	return 0;
}
