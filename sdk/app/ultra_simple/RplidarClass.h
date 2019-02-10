#pragma once

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "C:\usr\include\GregUtils\strguple.h"
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>


//using namespace rp::standalone::rplidar;

//namespace rp {
//	namespace  standalone {
//		namespace rplidar {
//			struct RPlidarDriver;
//		}
//	}
//};
//
//namespace boost {
//	struct thread;
//}
//		
#include "RPlidarNeedfullsForDLL.h"





struct RplidarReadingQueue {
	
	




	static std::pair<float, float> GetLinearFit(std::vector<rp::point> pData)
	{
		float xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;

		//	std::vector<float> xData;

			/*for (long i = 0; i < pData.size(); i++)
			{
				xData.push_back(pData[i].x);
			}*/
		auto sz = pData.size();

		for (long i = 0; i < sz; i++)
		{
			xSum += pData[i].x;
			ySum += pData[i].y;
			xxSum += pData[i].x * pData[i].x;
			xySum += pData[i].x * pData[i].y;
		}
		slope = (sz * xySum - xSum * ySum) / (sz * xxSum - xSum * xSum);
		intercept = (ySum - slope * xSum) / sz;
		/*std::vector<float> res;
		res.push_back(slope);
		res.push_back(intercept);*/
		return { slope,intercept };
	}


		//redeclared the rplidar sdk struct here so we don't have to bring in the rplidar sdk headers, etc.

	

	float fromRadial;
	float toRadial;
	bool keepGoing = true;
	bool useRangeFilter = false;
	_u32 baud;
	rp::standalone::rplidar::RPlidarDriver * drv = nullptr;
	char *     opt_com_path;
	static boost::thread * scanThread;
	boost::mutex qMutex;


	rp::enumLidarStatus lidarStatus=rp::STOPPED;

	boost::circular_buffer<rp::measure> * cb = nullptr;
	RplidarReadingQueue(float fromRadial, float toRadial, int qSize, _u32 baud = 256000, char * opt_com_path = (char*)"\\\\.\\com3");

	RplidarReadingQueue(int size);

	void setRange(int from, int to);

	int getFromTo(rp::measure & m, int fromRadial, int toRadial);
	
	int getFront(rp::measure & m);

	bool isInRange(float theta);

	bool push(rp::measure &m);

	~RplidarReadingQueue();

	void stop();

	rp::enumLidarStatus getLidarStatus();


	bool initLidat();

	bool run();

	bool runThreaded();

	void join();

};



#define INIT_RPLIDAR boost::thread * RplidarReadingQueue::scanThread = nullptr;