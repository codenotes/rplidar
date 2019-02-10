#pragma once

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>


#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "C:\usr\include\GregUtils\strguple.h"
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>



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
	
typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )

namespace rp {

	struct point {
		float x;
		float y;
		float z;
	};


	struct measure{ // : _rplidar_response_measurement_node_t {

		__pragma(pack(push, 1))
		_u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
		_u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
		_u16   distance_q2;
		__pragma(pack(pop))

			char temp[512];

		const char * debugPrint();

		float distance();

		long double deg2rad(long double deg);

		long double rad2deg(long double rad);

		point convToCart(float r, float theta, float omega = 90.0f);

		float theta();
	};
}



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


	boost::circular_buffer<rp::measure> * cb = nullptr;
	RplidarReadingQueue(float fromRadial, float toRadial, int qSize, _u32 baud = 256000, char * opt_com_path = (char*)"\\\\.\\com3");

	RplidarReadingQueue(int size);

	void setRange(int from, int to);

	void get(rp::measure & m, int fromRadial, int toRadial);
	 

	bool isInRange(float theta);

	bool push(rp::measure &m);

	~RplidarReadingQueue();

	void stop();

	


	bool initLidat();

	bool run();

	bool runThreaded();

	void join();

};



#define INIT_RPLIDAR boost::thread * RplidarReadingQueue::scanThread = nullptr;