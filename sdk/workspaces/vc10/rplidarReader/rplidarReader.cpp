// rplidarReader.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

#include "..\ultra_simple\RplidarClass.h"

//#define DLL_EXPORT __declspec(dllexport)

RplidarReadingQueue * gpRPInstance = nullptr;

extern "C"
{
	DLL_EXPORT int  test(void) { return 42; }
	DLL_EXPORT int StartLidar(float fromRadial, float toRadial, int qSize){
		SGUP_ODS(__FUNCTION__)

		if (gpRPInstance == nullptr)
			gpRPInstance = new RplidarReadingQueue(fromRadial, toRadial, qSize);

		gpRPInstance->setRange(fromRadial, toRadial);
		gpRPInstance->runThreaded();
		return 0;
	}

	DLL_EXPORT int StartLidarWithParams(float fromRadial, float toRadial, int qSize,
		int baud,  char* comport) {
		SGUP_ODS(__FUNCTION__)

			if (gpRPInstance == nullptr)
				gpRPInstance = new RplidarReadingQueue(fromRadial, toRadial, qSize,baud,
				(char*)comport);

		gpRPInstance->runThreaded();
		return 0;
	}

	DLL_EXPORT int StopLidar(void) {

		SGUP_ODS(__FUNCTION__)

		if (gpRPInstance) {
			gpRPInstance->stop();

			while (gpRPInstance->getLidarStatus() != rp::STOPPED) {
		//		SGUP_ODS(__FUNCTION__, "waiting until status becomes stopped!")
			}

			SGUP_ODS(__FUNCTION__,"status is STOPPED")

			delete gpRPInstance;
			gpRPInstance = nullptr;

		}
		return 0;
	}


	DLL_EXPORT int GetMeasure(rp::measure &m) {
		int ret;
		if (gpRPInstance) {

			ret=gpRPInstance->getFront(m);

		}
		else
			return -1;

	}

	DLL_EXPORT rp::enumLidarStatus GetLidarStatus() {
		
		if (gpRPInstance)
			return gpRPInstance->getLidarStatus();
		else
			return rp::UNKNOWN;
	}

}
