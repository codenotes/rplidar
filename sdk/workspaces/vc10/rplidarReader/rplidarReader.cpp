// rplidarReader.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

#include "..\ultra_simple\RplidarClass.h"
#include <chrono>
#include <thread>
#include "c:/usr/include/gregutils/Sqlbuilder.h"
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
		int baud,  const char* comport) {
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

	DLL_EXPORT void GetScan(rp::RplidarProxy::ScanVecType2 ** theScan){
		
		if (gpRPInstance)
			gpRPInstance->getScan(theScan);

		

	}

	DLL_EXPORT void GetScanWithExpiry(rp::RplidarProxy::ScanVecType2 ** theScan, int NewerThanMsec) {


	//	SGUP_ODSA(__FUNCTION__)

		if (gpRPInstance) {
			gpRPInstance->getScan(theScan,NewerThanMsec);
	

		}



	}


	DLL_EXPORT void DestroyScan(rp::RplidarProxy::ScanVecType2 ** theScan) {

		delete *theScan;
		//if (gpRPInstance)
			//gpRPInstance->getScan(theScan);
	}

	DLL_EXPORT void DumpScanToFile(std::string &fname ,	rp::RplidarProxy::ScanVecType2 * theScan, bool append) {

		if (gpRPInstance) {
			gpRPInstance->dumpScanToFile(fname ,theScan, append);
		}
			//if (gpRPInstance)

	}

	DLL_EXPORT void SavePresentScan(int id, std::string & database, rp::RplidarProxy::ScanVecType2 * theScan, float tilt) {


		if (gpRPInstance) {
			gpRPInstance->savePresentScan(id,database, theScan);
		}

	}


	DLL_EXPORT bool SendSQL(std::string & path, std::shared_ptr<std::string> & sql)
	{
	//	if (gpRPInstance) {
			return gpRPInstance->sendSQL(path, sql);
		//}

	}


	DLL_EXPORT void SetTiltLidar(float tilt)
	{

		if (gpRPInstance) {
			gpRPInstance->setTiltLidar(tilt);
		}

	}


	DLL_EXPORT int  SVToString(rp::RplidarProxy::ScanVecType2 * theScan, std::stringstream  & ss) {
		
		if (theScan == nullptr) {

			SGUP_ODSA(__FUNCTION__, "no scan sent in...");
			return 0;
		}


		for (auto &p : *theScan) {
			
			ss << p.first << ":   ," << p.second.distance << "  ,  " <<p.second.tilt   << std::endl;
		}

		return theScan->size();

	}

	DLL_EXPORT int  GetScanFromDatabase(rp::RplidarProxy::ScanVecType2 ** psv, std::string & path, std::optional<int> & id) {

		return RplidarReadingQueue::getScanFromDatabase(psv, path, id);


	}

	DLL_EXPORT int  SaveScanToDatabase(rp::RplidarProxy::ScanVecType2 * psv, std::string & path, std::optional<int> & id) {

		return RplidarReadingQueue::saveScanToDatabase(psv, path, id);


	}

	DLL_EXPORT int  GetRangeOfScansFromDatabase(rp::RplidarProxy::ScanVecType2 ** sv, std::string & path, 
		rp::RplidarProxy::scanRange rng, bool loop, bool reset) {
	//										
		static rp::RplidarProxy::scanRange theRange;
		static int lastIndex = -1;

		SQLBuilder  sb;
		sb.createOrOpenDatabase(path);


		if (reset) { theRange = std::nullopt; lastIndex = -1; SGUP_ODSA(__FUNCTION__, "reset called"); }

		if (!theRange) //this is the first call of the function, because theRange is static optional and has not been initialized
		{
			SGUP_ODSA(__FUNCTION__, "First call");

			if (rng) //we are intended to move through a range they gave us
			{
				SGUP_ODSA(__FUNCTION__, "Setting range passed in:", rng->first, rng->second);
				theRange = rng;
			}
			else { //means we should go LIFO from max to min of whatever is in the database
				//theRange->first= 
				auto sql = "select ifnull(max(id),0),ifnull(min(id),0) from sweep;";
				sb.execSQL(sql);
				theRange->first = std::stoi(sb.results[0][0].second);
				theRange->second = std::stoi(sb.results[0][1].second);
				SGUP_ODSA(__FUNCTION__, "max:", theRange->first, "min:", theRange->second);

				
			}
		}
		else //repeated call where theRange is properly set, ie, theRAnge IS valid
		{
			

			auto sql =
				boost::format("select id, angle, distance, tilt from sweep where id == %1%") % theRange->first--; //count backward from max

			*sv = new rp::RplidarProxy::ScanVecType2;

			
			sqlite3_stmt *statement;

			if (sqlite3_prepare_v2(sb.gdb, sql.str().c_str(), -1, &statement, 0) == SQLITE_OK)
			{
				int cols = sqlite3_column_count(statement);
				int result = 0;
				while (true)
				{
					result = sqlite3_step(statement);

					if (result == SQLITE_ROW)
					{

						auto id =		sqlite3_column_int(statement, 0);
						auto angle =	sqlite3_column_double(statement, 1);
						auto distance = sqlite3_column_int(statement, 2 );
						auto tilt =		sqlite3_column_double(statement, 3);

						(*sv)->push_back({ angle, rp::beam((_u16)distance, tilt) });

					}
					else
					{
						break;
					}
				}

				sqlite3_finalize(statement);
			}

		}

		return 0;
	}



}

#if 0


#endif