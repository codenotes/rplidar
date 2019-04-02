// rplidarReader.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

#include "..\ultra_simple\RplidarClass.h"
#include <chrono>
#include <thread>
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
				comport);

		SGUP_ODSA(__FUNCTION__,"COMPORT RECEIVED:", comport)

		gpRPInstance->runThreaded();
		return 0;
	}

	DLL_EXPORT int StopLidar(void) {

		SGUP_ODS(__FUNCTION__)

		if (gpRPInstance) {
			gpRPInstance->stop();

			while (gpRPInstance->getLidarStatus() != rp::STOPPED) {
				OutputDebugStringA(__FUNCTION__);
		//		SGUP_ODS(__FUNCTION__, "waiting until status becomes stopped!")
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}

			SGUP_ODS(__FUNCTION__, "status is STOPPED")


				
			


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

	DLL_EXPORT void GetScan(rp::RplidarProxy::ScanVecType ** theScan){
		
		if (gpRPInstance)
			gpRPInstance->getScan(theScan);

		

	}

	DLL_EXPORT void GetScanWithExpiry(rp::RplidarProxy::ScanVecType ** theScan, int NewerThanMsec) {


		SGUP_ODS(__FUNCTION__)

		if (gpRPInstance) {
			gpRPInstance->getScan(theScan);

			return;
			for (rp::RplidarProxy::ScanVecType::const_iterator itr = (**theScan).cbegin(); itr != (**theScan).cend(); ) {
			
				auto born = itr->second.second;
				auto lived = std::chrono::duration_cast<std::chrono::milliseconds>(rp::Clock::now() - born).count();

				if (lived > NewerThanMsec) { //it is old, delete it
					(**theScan).erase(itr);
					SGUP_ODS(__FUNCTION__,"old ray detected, deleting:",itr->first, itr->second.first) //angle/distance
				}
				else
				{
					std::next(itr);
				}

				
			}

		

		}



	}


	DLL_EXPORT void DestroyScan(rp::RplidarProxy::ScanVecType ** theScan) {

		delete *theScan;
		//if (gpRPInstance)
			//gpRPInstance->getScan(theScan);
	}

	DLL_EXPORT void DumpScanToFile(std::string &fname ,	rp::RplidarProxy::ScanVecType * theScan, bool append) {

		if (gpRPInstance) {
			gpRPInstance->dumpScanToFile(fname ,theScan, append);
		}
			//if (gpRPInstance)

	}

}
