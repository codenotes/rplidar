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

	static  bool init( std::map<std::string, std::string> args,  std::string  nodeName) {
		ros::init(args, nodeName);

		if (ros::master::check()) {

			ROSStuff::nodeHandle.reset(new ros::NodeHandle); //this starts everything and connects to master


			ROSStuff::isInitialized = true;
		}
		else {
			SGUP_ODSA(__FUNCTION__, "error master check returns false...");
			return false;
		}

	}


	static void stopSub() {

		if (!isSubscribing) {
			SGUP_ODSA(__FUNCTION__, "stopped subscription but wasn't subscribing in first place. ");
			return;
		}


		theSubscriber->shutdown();
		isSubscribing = false;
	}

	static void initSub(const std::string & topic, int qSize) {

		if (!isInitialized) {
			SGUP_ODSA(__FUNCTION__, "ROS not initialized!");
			return;
		}

		cb2.reset( new CircBufferScanVecT(CB2_SIZE) );

		*theSubscriber = ROSStuff::nodeHandle->subscribe<sensor_msgs::LaserScan>(topic, qSize, subCallback);


		ROSStuff::isSubscribing = true;
	}
#if 0
	int main(int argc, char * argv[]) {
		ros::init(argc, argv, "rplidar_node");

		std::string channel_type;
		std::string tcp_ip;
		std::string serial_port;
		int tcp_port = 20108;
		int serial_baudrate = 115200;
		std::string frame_id;
		bool inverted = false;
		bool angle_compensate = true;
		float max_distance = 8.0;
		int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
		std::string scan_mode;
		ros::NodeHandle nh;
		ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
		ros::NodeHandle nh_private("~");
		nh_private.param<std::string>("channel_type", channel_type, "serial");
		nh_private.param<std::string>("tcp_ip", tcp_ip, "192.168.0.7");
		nh_private.param<int>("tcp_port", tcp_port, 20108);
		nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
		nh_private.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
		nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
		nh_private.param<bool>("inverted", inverted, false);
		nh_private.param<bool>("angle_compensate", angle_compensate, false);
		nh_private.param<std::string>("scan_mode", scan_mode, std::string());

		//ROS_INFO("RPLIDAR running on ROS package rplidar_ros. SDK Version:"RPLIDAR_SDK_VERSION"");

		u_result     op_result;

		// create the driver instance
		//if (channel_type == "tcp") {
		//	drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
		//}
		//else {
		//	drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
		//}


		//if (!drv) {
		//	ROS_ERROR("Create Driver fail, exit");
		//	return -2;
		//}

		//if (channel_type == "tcp") {
		//	// make connection...
		//	if (IS_FAIL(drv->connect(tcp_ip.c_str(), (_u32)tcp_port))) {
		//		ROS_ERROR("Error, cannot bind to the specified serial port %s.", serial_port.c_str());
		//		RPlidarDriver::DisposeDriver(drv);
		//		return -1;
		//	}

		//}
		//else {
		//	// make connection...
		//	if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
		//		ROS_ERROR("Error, cannot bind to the specified serial port %s.", serial_port.c_str());
		//		RPlidarDriver::DisposeDriver(drv);
		//		return -1;
		//	}

		//}

		//// get rplidar device info
		//if (!getRPLIDARDeviceInfo(drv)) {
		//	return -1;
		//}

		//// check health...
		//if (!checkRPLIDARHealth(drv)) {
		//	RPlidarDriver::DisposeDriver(drv);
		//	return -1;
		//}

		//ros::ServiceServer stop_motor_service = nh.advertiseService("stop_motor", stop_motor);
		//ros::ServiceServer start_motor_service = nh.advertiseService("start_motor", start_motor);

		//drv->startMotor();

		RplidarScanMode current_scan_mode;
		if (scan_mode.empty()) {
			op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
		}
		else {
			std::vector<RplidarScanMode> allSupportedScanModes;
			op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

			if (IS_OK(op_result)) {
				_u16 selectedScanMode = _u16(-1);
				for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
					if (iter->scan_mode == scan_mode) {
						selectedScanMode = iter->id;
						break;
					}
				}

				if (selectedScanMode == _u16(-1)) {
					ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
					for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
						ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode,
							iter->max_distance, (1000 / iter->us_per_sample));
					}
					op_result = RESULT_OPERATION_FAIL;
				}
				else {
					op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
				}
			}
		}

		if (IS_OK(op_result))
		{
			//default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
			angle_compensate_multiple = (int)(1000 * 1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
			if (angle_compensate_multiple < 1)
				angle_compensate_multiple = 1;
			max_distance = current_scan_mode.max_distance;
			ROS_INFO("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", current_scan_mode.scan_mode,
				current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample), angle_compensate_multiple);
		}
		else
		{
			ROS_ERROR("Can not start scan: %08x!", op_result);
		}

		ros::Time start_scan_time;
		ros::Time end_scan_time;
		double scan_duration;
		while (ros::ok()) {
			rplidar_response_measurement_node_hq_t nodes[360 * 8];
			size_t   count = _countof(nodes);

			start_scan_time = ros::Time::now();
			op_result = drv->grabScanDataHq(nodes, count);
			end_scan_time = ros::Time::now();
			scan_duration = (end_scan_time - start_scan_time).toSec();

			if (op_result == RESULT_OK) {
				op_result = drv->ascendScanData(nodes, count);
				float angle_min = DEG2RAD(0.0f);
				float angle_max = DEG2RAD(359.0f);
				if (op_result == RESULT_OK) {
					if (angle_compensate) {
						//const int angle_compensate_multiple = 1;
						const int angle_compensate_nodes_count = 360 * angle_compensate_multiple;
						int angle_compensate_offset = 0;
						rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
						memset(angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));

						int i = 0, j = 0;
						for (; i < count; i++) {
							if (nodes[i].dist_mm_q2 != 0) {
								float angle = getAngle(nodes[i]);
								int angle_value = (int)(angle * angle_compensate_multiple);
								if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
								for (j = 0; j < angle_compensate_multiple; j++) {
									angle_compensate_nodes[angle_value - angle_compensate_offset + j] = nodes[i];
								}
							}
						}

						publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
							start_scan_time, scan_duration, inverted,
							angle_min, angle_max, max_distance,
							frame_id);
					}
					else {
						int start_node = 0, end_node = 0;
						int i = 0;
						// find the first valid node and last valid node
						while (nodes[i++].dist_mm_q2 == 0);
						start_node = i - 1;
						i = count - 1;
						while (nodes[i--].dist_mm_q2 == 0);
						end_node = i + 1;

						angle_min = DEG2RAD(getAngle(nodes[start_node]));
						angle_max = DEG2RAD(getAngle(nodes[end_node]));

						publish_scan(&scan_pub, &nodes[start_node], end_node - start_node + 1,
							start_scan_time, scan_duration, inverted,
							angle_min, angle_max, max_distance,
							frame_id);
					}
				}
				else if (op_result == RESULT_OPERATION_FAIL) {
					// All the data is invalid, just publish them
					float angle_min = DEG2RAD(0.0f);
					float angle_max = DEG2RAD(359.0f);

					publish_scan(&scan_pub, nodes, count,
						start_scan_time, scan_duration, inverted,
						angle_min, angle_max, max_distance,
						frame_id);
				}
			}

			ros::spinOnce();
		}

		// done!
		drv->stop();
		drv->stopMotor();
		RPlidarDriver::DisposeDriver(drv);
		return 0;
	}
#endif
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

	static void spinThread(int doEveryMsec) {

		SGUP_ODSA(__FUNCTION__, "ROS spin thread starting.");
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
boost::mutex  ROSStuff::qMutex;

#ifndef ROSLIBS_LINKED
#define ROSLIBS_LINKED
#pragma comment(lib, "roscpp.lib")
#pragma comment(lib, "rosconsole.lib")
#pragma comment(lib, "roscpp_serialization.lib")
#pragma comment(lib, "rostime.lib")
#endif