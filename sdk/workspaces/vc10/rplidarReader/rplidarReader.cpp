// rplidarReader.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"

#include "..\ultra_simple\RplidarClass.h"
#include <chrono>
#include <thread>
#include "c:/usr/include/gregutils/Sqlbuilder.h"
#include "/usr/include/rplidar/rosstuff.h"
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

		if (*theScan==nullptr) {
			SGUP_ODSA(__FUNCTION__, "deleting a *scan that is nullptr! returning...");
			return;
		}

		if (theScan == nullptr) {
			SGUP_ODSA(__FUNCTION__, "deleting a scan that is nullptr! returning...");
			return;
		}



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
			gpRPInstance->savePresentScan(id,database, theScan, "sweep");
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

	DLL_EXPORT int  GetScanFromDatabase(rp::RplidarProxy::ScanVecType2 ** psv, std::string & path, std::optional<int> & id,
		std::optional<std::string> & sweepTableName) {

		return RplidarReadingQueue::getScanFromDatabase(psv, path, id, sweepTableName);


	}

	DLL_EXPORT int  SaveScanToDatabase(rp::RplidarProxy::ScanVecType2 * psv, std::string & path, std::optional<int> & id,
		std::optional<std::string> & sweepTableName) {

		return RplidarReadingQueue::saveScanToDatabase(psv, path, id, sweepTableName);


	}

	DLL_EXPORT int  GetRangeOfScansFromDatabase(rp::RplidarProxy::ScanVecType2 ** sv, std::string & path, 
		rp::RplidarProxy::scanRange rng, bool loop, bool reset, std::optional<std::string> & sweepTableName) {
	//										
		static rp::RplidarProxy::scanRange theRange=std::nullopt;
		std::stringstream ss;

		std::string theSweepTableName;
		if (sweepTableName) {
			theSweepTableName = *sweepTableName;
		}
		else
			theSweepTableName = "sweep";

		SGUP_ODSA(__FUNCTION__, "!!!hasvalue:", theRange.has_value());

		SQLBuilder  sb;
		sb.createOrOpenDatabase(path);

		auto rst = [&]() {
			theRange = std::nullopt;
			*sv = nullptr;
			SGUP_ODSA(__FUNCTION__, "reset called");
			//return -1;
		};

		if (reset) { rst();   return -1; }

		

		if (theRange.has_value()==false) //this is the first call of the function, because theRange is static optional and has not been initialized
		{
			SGUP_ODSA(__FUNCTION__, "HASVAL:",theRange.has_value(), "First call");

			if (rng) //we are intended to move through a range they gave us
			{
				SGUP_ODSA(__FUNCTION__, "Setting range passed in:", rng->first, rng->second);
				theRange = rng;
				*sv = nullptr;// don't expect a result set because this is first call
			}
			else { //means we should go LIFO from max to min of whatever is in the database
				//theRange->first= 
				ss<< "select ifnull(max(id),0),ifnull(min(id),0) from "<<theSweepTableName<<";";
				auto b=sb.execSQL(ss.str());

				if (!b)
				{
					SGUP_ODSA(__FUNCTION__, "error in tablename", theSweepTableName);
					*sv = nullptr;
					return 1;
				}

				theRange = std::pair<int,int>(std::stoi(sb.results[0][0].second), std::stoi(sb.results[0][1].second));
				//theRange->first = std::stoi(sb.results[0][0].second);
				//theRange->second = std::stoi(sb.results[0][1].second);
				SGUP_ODSA(__FUNCTION__, "max:", theRange->first, "min:", theRange->second);
				*sv = nullptr; 
				
			}
		}
		else //repeated call where theRange is properly set, ie, theRAnge IS valid
		{
			//if max < min, because I decrement theRange->first and use it as a last index
			if (theRange->first < theRange->second) {
				SGUP_ODSA(__FUNCTION__, "past minimum, returning, should either reset or loop");
				*sv = nullptr;

				if (loop) {
					rst();
				}

				
			}

			auto sql =
				boost::format("select  id, angle, distance, tilt from %1% where id == %2%") %theSweepTableName % theRange->first--; //count backward from max

			*sv = new rp::RplidarProxy::ScanVecType2;

			
			sqlite3_stmt *statement;

			if (sqlite3_prepare_v2(sb.gdb, sql.str().c_str(), -1, &statement, 0) == SQLITE_OK)
			{
				//int cols = sqlite3_column_count(statement);
				int result = 0;
				//int id = -1;
				while (true)
				{
					result = sqlite3_step(statement);

					if (result == SQLITE_ROW)
					{

						auto id =		sqlite3_column_int(statement, 0);
						auto angle =	sqlite3_column_double(statement, 1);
						auto distance = sqlite3_column_int(statement, 2 );
						auto tilt =		sqlite3_column_double(statement, 3);
						//auto cnt =		sqlite3_column_double(statement, 4);

						(*sv)->push_back({ angle, rp::beam((_u16)distance, tilt,id) });

					}
					else if (result == SQLITE_DONE) {

						//SGUP_ODSA(__FUNCTION__, "SQLITE_DONE, at id:", id);

						break;
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

	DLL_EXPORT  bool ROSAction(std::map<std::string, std::string> & args, int qSize, rp::enumROSCommand command)
	{
		int spinMsec = 100;
		auto it = args.find("spinMsec");
		auto topic = args.find("topic");

		switch (command)
		{
		case rp::START_SUB:
			if (topic != args.end()) {
				ROSStuff::startSub(args["topic"], qSize);
			}
			else
			{
				SG2("START_SUB, but no topic!")
			}
			break;
		case rp::STOP_SUB:
			ROSStuff::stopSub();

			break;
		case rp::INTITIALIZE:
		
			if (it != args.end())
			{
				spinMsec = std::stoi(it->second);
			}

			return ROSStuff::init(args, "rplidar", std::optional<int>(spinMsec));


			break;

		case rp::TICK_SPIN:
			if(ROSStuff::isSubscribing)
				ros::spinOnce();

			break;
		default:
			break;
		}

		return true;
	}

	DLL_EXPORT  void GetROSScan(rp::RplidarProxy::ScanVecType2 ** theScan) {
		ROSStuff::GetROSScan(theScan);
	}


}

