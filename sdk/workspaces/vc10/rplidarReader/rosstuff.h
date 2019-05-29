#pragma once
#include "..\..\..\app\ultra_simple\RplidarClass.h"
//ros crap warnings
#pragma warning(disable:4267 4244 4996) 

#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <boost/circular_buffer.hpp>
#include <boost/thread/thread.hpp>


#define RAD2DEG(x) ((x)*180./M_PI)

//#include "C:/usr/include/rplidar/RPlidarNeedfullsForDLL.h"

#define M_PI 3.14159f
#define CB2_SIZE 30

struct ROSStuff
{


	using CircBufferScanVecT = boost::circular_buffer< rp::RplidarProxy::ScanVecType2>;
	static std::shared_ptr < ros::NodeHandle> nodeHandle;
	static std::shared_ptr < ros::Publisher> thePublisher;
	static std::shared_ptr < ros::Subscriber> theSubscriber;
	static std::shared_ptr<ros::Rate> loop_rate;
	static bool isInitialized;	
	static bool isSubscribing;
	static std::shared_ptr<CircBufferScanVecT> cb2;
	static boost::mutex qMutex;
	static std::shared_ptr<boost::thread> mThreadPtr;

	//dont use, for reference
	static void publish_scan(rplidar_response_measurement_node_hq_t *nodes,
		size_t node_count, ros::Time start,
		double scan_time, bool inverted,
		float angle_min, float angle_max,
		float max_distance,
		std::string frame_id)
	{

		if (!isInitialized) {
			SGUP_ODSA(__FUNCTION__, "not insitialized");
			return;
		}
		static int scan_count = 0;
		sensor_msgs::LaserScan scan_msg;

		scan_msg.header.stamp = start;
		scan_msg.header.frame_id = frame_id;
		scan_count++;

		bool reversed = (angle_max > angle_min);
		if (reversed) {
			scan_msg.angle_min = M_PI - angle_max;
			scan_msg.angle_max = M_PI - angle_min;
		}
		else {
			scan_msg.angle_min = M_PI - angle_min;
			scan_msg.angle_max = M_PI - angle_max;
		}
		scan_msg.angle_increment =
			(scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

		scan_msg.scan_time = scan_time;
		scan_msg.time_increment = scan_time / (double)(node_count - 1);
		scan_msg.range_min = 0.15;
		scan_msg.range_max = max_distance;//8.0;

		scan_msg.intensities.resize(node_count);
		scan_msg.ranges.resize(node_count);
		bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
		if (!reverse_data) {
			for (size_t i = 0; i < node_count; i++) {
				float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
				if (read_value == 0.0)
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
				else
					scan_msg.ranges[i] = read_value;
				scan_msg.intensities[i] = (float)(nodes[i].quality >> 2);
			}
		}
		else {
			for (size_t i = 0; i < node_count; i++) {
				float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;
				if (read_value == 0.0)
					scan_msg.ranges[node_count - 1 - i] = std::numeric_limits<float>::infinity();
				else
					scan_msg.ranges[node_count - 1 - i] = read_value;
				scan_msg.intensities[node_count - 1 - i] = (float)(nodes[i].quality >> 2);
			}
		}

		thePublisher->publish(scan_msg);
	}

	static  bool init( std::map<std::string, std::string> args,  std::string  nodeName, std::optional<int> startSpinner) {
		ros::init(args, nodeName);

		if (ros::master::check()) {

			SGUP_ODSA(__FUNCTION__, "ROS_MASTER exists!");
			ROSStuff::nodeHandle.reset(new ros::NodeHandle); //this starts everything and connects to master

			theSubscriber.reset(new ros::Subscriber);
			ROSStuff::isInitialized = true;
		}
		else {
			SGUP_ODSA(__FUNCTION__, "error master check returns false...");
			return false;
		}

		if (startSpinner) {
			startSpin(*startSpinner);
		}

	}

	static void startSpin(int everyMsec) {
		mThreadPtr.reset(new boost::thread(spinThread, everyMsec));
	
	}



	static void stopSub() {

		if (!isSubscribing) {
			SGUP_ODSA(__FUNCTION__, "stopped subscription but wasn't subscribing in first place. ");
			return;
		}


		theSubscriber->shutdown();
		isSubscribing = false;
	}

	static void startSub(const std::string & topic, int qSize) {

		if (!isInitialized) {
			SGUP_ODSA(__FUNCTION__, "ROS not initialized!");
			return;
		}

		cb2.reset( new CircBufferScanVecT(CB2_SIZE) );

		*theSubscriber = ROSStuff::nodeHandle->subscribe<sensor_msgs::LaserScan>(topic, qSize, subCallback);


		ROSStuff::isSubscribing = true;
	}

	static void GetROSScan(rp::RplidarProxy::ScanVecType2 ** sp) {

		auto sz = cb2->size();

		if (!isSubscribing || !cb2 || !sz) {
			SGUP_ODSA(__FUNCTION__, "GETTING A SCAN WHEN THERE IS NOTHING IN Q OR NO Q OR NOT SUBSCRIBING OR EMPTY Q");
			return;
		}

		qMutex.lock();
		*sp = new rp::RplidarProxy::ScanVecType2(cb2->front());
		cb2->pop_front();
		qMutex.unlock();

	}

	static void stopSpin() {
		if (mThreadPtr) {
			mThreadPtr->interrupt();
			mThreadPtr.reset();
		}
	}

	static void spinThread(int doEveryMsec) {

		SGUP_ODSA(__FUNCTION__, "ROS spin thread starting.");

		if (!ros::isInitialized()) {
			SG2("ROS Not intialized!");
			return;
		}

		try {

			ros::spinOnce();
			boost::this_thread::sleep(boost::posix_time::milliseconds(doEveryMsec));
			boost::this_thread::interruption_point();
		}
		catch (boost::thread_interrupted&)
		{
			SGUP_ODSA(__FUNCTION__, "ROS spin thread interrupted. Exiting.");
		}

	
	}

	static void subCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
		rp::RplidarProxy::ScanVecType2 sv;
		_u16 rng = 0;

		int count = scan->scan_time / scan->time_increment;
		//ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
	//	ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));


		for (int i = 0; i < count; i++) {
			float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
			if (std::numeric_limits<float>::infinity() == scan->ranges[i]) {
				rng = 0;
			}
			else {
				rng = boost::lexical_cast<_u16>(scan->ranges[i]);
			}

			sv.push_back({ degree  ,rp::beam{ rng, scan->scan_time } });
		//	ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
		}

		qMutex.lock();
		cb2->push_back(sv);
		qMutex.unlock();

	}

	//int subscribe(int argc, char **argv)
	//{
	//	ros::init(argc, argv, "rplidar_node_client");
	//	ros::NodeHandle n;

	//	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, subCallback);

	//	ros::spin();

	//	return 0;
	//}
};

#define ROS_INIT \
std::shared_ptr<ros::NodeHandle > ROSStuff::nodeHandle;\
std::shared_ptr<ros::Publisher>  ROSStuff::thePublisher;\
std::shared_ptr<ros::Subscriber>  ROSStuff::theSubscriber;\
std::shared_ptr<ros::Rate> ROSStuff::loop_rate; \
bool ROSStuff::isInitialized = false; \
bool ROSStuff::isSubscribing = false; \
std::shared_ptr<ROSStuff::CircBufferScanVecT>  ROSStuff::cb2; \
boost::mutex  ROSStuff::qMutex; \
std::shared_ptr<boost::thread> ROSStuff::mThreadPtr;

#ifndef ROSLIBS_LINKED
#define ROSLIBS_LINKED
#pragma comment(lib, "roscpp.lib")
#pragma comment(lib, "rosconsole.lib")
#pragma comment(lib, "roscpp_serialization.lib")
#pragma comment(lib, "rostime.lib")
#endif