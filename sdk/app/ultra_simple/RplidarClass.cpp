
#include "RplidarClass.h"

//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
//#include "C:\usr\include\GregUtils\strguple.h"
//#include <boost/circular_buffer.hpp>
//#include <boost/thread.hpp>
#include "windows.h"
#include <chrono>
#include <thread>

#include "c:/usr/include/gregutils/Sqlbuilder.h"

INIT_RPLIDAR
INIT_STRGUPLE

RplidarReadingQueue::RplidarReadingQueue(float fromRadial, float toRadial, int qSize,
	_u32 baud /*= 256000*/,const char * opt_com_path /*= "\\\\.\\com3"*/) :fromRadial(fromRadial), toRadial(toRadial), baud(baud), opt_com_path(opt_com_path), useRangeFilter(true)
{
	SGUP_ODSA(__FUNCTION__, "COMPORT RECEIVED:", opt_com_path)
	cb = new boost::circular_buffer<rp::measure>(qSize);
}

RplidarReadingQueue::RplidarReadingQueue(int size)
{
	cb = new boost::circular_buffer<rp::measure>(size);
}

void RplidarReadingQueue::setRange(int from, int to)
{
	fromRadial = from;
	toRadial = to;

	if (from == to) {
		useRangeFilter = false;
	}
	else
	useRangeFilter = true;
}

int RplidarReadingQueue::getFromTo(rp::measure & m, int fromRadial, int toRadial)
{
	//boost::mutex::scoped_lock lock(qMutex);
	return -1;
}

int RplidarReadingQueue::getFront(rp::measure & m)
{
	//boost::mutex::scoped_lock lock(qMutex);

	if (cb->empty())
		return 0;

	m = cb->front();

	return cb->size();

}

bool RplidarReadingQueue::isInRange(float theta)
{
	//	float minTheta = std::min(fromRadial, toRadial);
	//	float maxTheta = std::max(fromRadial, toRadial);

	bool truth1 = (theta >= fromRadial) && (theta <= 359.99f);
	//or
	bool truth2 = (theta > 0.0f) && (theta <= toRadial);


	//	SGUP_ODS(__FUNCTION__, "from/to radials", fromRadial, toRadial, "theta:",theta, truth1,truth2,  (truth1 || truth2)?"TRUE":"FALSE"    );


	return (truth1 || truth2);

	//sweep to the right so
	//we must be greater than minTheta up to 359.999
}

//boost::mutex boobs;



bool RplidarReadingQueue::push(rp::measure &m)
{
//	boost::mutex::scoped_lock lock(qMutex);
	//SGUP_ODS(__FUNCTION__, "acquiring lock")

	auto fnConv=[&](_u16 angle, _u16 dist, rp::Clock::time_point tp) {
		auto p = rp::measure::convertAngleDist(angle, dist);
		auto tkey = roundf(p.first * 10) / 10;
		storedRead[tkey] = { p.second,tp };
	};

	if (m.distance() == 0) return false;

	auto thet = m.theta();
	m.born = rp::Clock::now();


	if (useRangeFilter)
		if (isInRange(thet)) {
		//	OutputDebugStringA(__FUNCTION__ " start");
			qMutex.lock();
			cb->push_back(m);

			fnConv(m.angle_q6_checkbit, m.distance_q2, m.born);

			//storedRead[m.angle_q6_checkbit] = m.distance_q2;
	//		OutputDebugStringA(__FUNCTION__ " end");
			qMutex.unlock();
		//	SGUP_ODS("PUSH THETA:", thet);
	
			return true;
		}
		else {
			//		SGUP_ODS("REJECT THETA:", thet);
		
			return false;
		}
	else {
//		OutputDebugStringA(__FUNCTION__ " start 2");
		qMutex.lock();
		cb->push_back(m);
		fnConv(m.angle_q6_checkbit, m.distance_q2, m.born);
		//storedRead[m.theta()] = m.distance();
	//	storedRead[m.angle_q6_checkbit] = m.distance_q2;
		qMutex.unlock();
	//	OutputDebugStringA(__FUNCTION__ " end 2");
	}

	//SGUP_ODS(__FUNCTION__, "release lock")
	return true;
}


void RplidarReadingQueue::getScan(rp::RplidarProxy::ScanVecType ** theScan, int msecExpiry)
{

	

	if (lidarStatus != rp::STARTED) {
		OutputDebugStringA(__FUNCTION__ " LIDAR not spooled up yet...returning...");
		*theScan = nullptr;
		return;
	}
	//SGUP_ODSA(__FUNCTION__, "acquiring lock")
	qMutex.lock();
	//theScan = storedRead;
	//<angle  <distance, timepoint   >>    
	*theScan = new rp::RplidarProxy::ScanVecType();

	if (msecExpiry == 0) {
		//(*theScan)->push_back({ 2.2,3.3 });
		(*theScan)->assign(storedRead.begin(), storedRead.end());
		//theScan.push_back({ 2.2,3.4 });
		storedRead.clear();
	}
	else { //we have an expiry time

		for (auto &x : storedRead) {

			auto born = RP_GET_EXPIRY(x);
			auto lived = std::chrono::duration_cast<std::chrono::milliseconds>(rp::Clock::now() - born).count();

			if (lived > msecExpiry) { //it is old, skip it
				//(**theScan).erase(itr);

				SGUP_ODS(__FUNCTION__, "old ray detected, deleting:", x.first, x.second.first) //angle/distance
			}
			else
			{
				(*theScan)->push_back(x);

			//	std::next(itr);
			}
		}

		storedRead.clear();
	}
	//theScan = crap;
	//SGUP_ODSA(__FUNCTION__, "BOOBS!", theScan.size());
	qMutex.unlock();
//	SGUP_ODS(__FUNCTION__, "unlocked")
	
		//try {
	//	boobs.lock();
	//	theScan = storedRead;
	//	boobs.unlock();
	//}
	//catch (std::exception &e)
	//{
	//	SGUP_ODSA(__FUNCTION__, e.what());
	//}

	return;

	

	OutputDebugStringA(__FUNCTION__ " going to try and get lock");
	auto b = qMutex.try_lock();
	if (!b) {
		OutputDebugStringA(__FUNCTION__ " try and fail lock");
		return;
	}
	else
	{
//		theScan.assign(storedRead.begin(), storedRead.end());
		OutputDebugStringA(__FUNCTION__ " Got lock, unlocking");
		qMutex.unlock();

	}
	//qMutex.lock();
	
	OutputDebugStringA(__FUNCTION__ " end");
	//SGUP_ODS(__FUNCTION__, "releaseing lock")

}

RplidarReadingQueue::~RplidarReadingQueue()
{
	
	
	if (cb)
		delete cb;
	
	if(drv)
		delete drv;
	//drv->disconnect();
	//drv->DisposeDriver(drv);

//	RPlidarDriver::DisposeDriver(drv);
	
	drv = nullptr;
	
	
}

void RplidarReadingQueue::stop()
{
	SGUP_ODS(__FUNCTION__)
	keepGoing = false;
	scanThread->interrupt();
}


rp::enumLidarStatus RplidarReadingQueue::getLidarStatus()
{
	return lidarStatus;
}

bool RplidarReadingQueue::initLidat()
{
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;
	using namespace rp::standalone::rplidar;
	keepGoing = true;

//	opt_com_baudrate = baud;
	SGUP_ODSA(__FUNCTION__, "COMPORT RECEIVED:", opt_com_path)

	

		if (!drv) {
			SGUP_ODS(__FUNCTION__,"attempting to create LIDAR driver on serial port.")
			drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		}

	if (!drv) {
		SGUP_ODS("insufficent memory, exit");
		return false;
	}


	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...

	if (!drv)
		drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (IS_OK(drv->connect(opt_com_path.c_str(), baud)))
	{
		auto op_result = drv->getDeviceInfo(devinfo);

		if (IS_OK(op_result))
		{
			connectSuccess = true;
			SGUP_ODS("connect lidar successful");
		}
		else
		{
			SGUP_ODS("failure to connect lidar.");
			delete drv;
			drv = NULL;

		}
	}


	if (!connectSuccess) {
		SGUP_ODS("Error, cannot bind to the specified serial port", opt_com_path);
		return false;
	}

	SGUP_ODS("Firmware Ver: %d.%02d\n", devinfo.firmware_version >> 8
		, devinfo.firmware_version & 0xFF,
		"Hardware Rev: %d\n", (int)devinfo.hardware_version);


	return true;
}

bool RplidarReadingQueue::run()
{
	//		signal(SIGINT, ctrlc);
	//SGUP_ODS(__FUNCTION__)
	SGUP_ODS(__FUNCTION__, "COMPORT RECEIVED:", opt_com_path)

	if (!initLidat()) return false;

	SGUP_ODS(__FUNCTION__, "initdat worked?")

	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);

	lidarStatus = rp::STARTED;

	char temp[512];
	SGUP_ODS(__FUNCTION__, "keepgoing:", keepGoing);

	// fetech result and print it out...
	while (keepGoing && lidarStatus==rp::STARTED) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1)); //new
		rplidar_response_measurement_node_t nodes[8192];
		size_t   count = _countof(nodes);

		auto op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);
			for (int pos = 0; pos < (int)count; ++pos) {

				rp::measure m = (rp::measure&)nodes[pos];
				if (push(m)) {
					//	SGUP_ODS(m.debugPrint());
				}
				else {
					//	SGUP_ODS("REJECT:", m.theta());
				}


			}
		}
		else {
			SGUP_ODS(__FUNCTION__, "error:strange");
		}

		try {
			boost::this_thread::interruption_point();
		
		}
		catch (...)
		{
			OutputDebugStringA(__FUNCTION__ "interupt!");
			
			SGUP_ODS(__FUNCTION__, "thread interrupt");

			SGUP_ODS("Stopping scan and motor");

			OutputDebugStringA(__FUNCTION__ "about stop motors");

			drv->stop();
			drv->stopMotor();

			OutputDebugStringA(__FUNCTION__ "motors stopped!");
			lidarStatus = rp::STOPPED;
			break;
		}
	}

}



bool RplidarReadingQueue::runThreaded()
{
	//SGUP_ODS(__FUNCTION__)
	SGUP_ODSA(__FUNCTION__, "COMPORT RECEIVED:", opt_com_path)

	scanThread = new boost::thread(std::bind(&RplidarReadingQueue::run, this));
	return true;
}

void RplidarReadingQueue::join()
{
	scanThread->join();
}

void RplidarReadingQueue::dumpScanToFile(std::string &fname, rp::RplidarProxy::ScanVecType * theScan, bool append/*=false*/)
{
	std::fstream of;

	if(append)
		of.open(fname, std::fstream::out | std::fstream::app);
	else
		of.open(fname, std::fstream::out);

	of << "BOOBS" << std::endl;
	rp::point pnt;
	float x, y, z;
	using namespace std;

	for (auto &p : *theScan) {
		pnt=rp::measure::convToCart(p.second.first, p.first);
		of <<' '<<  p.first << ' ' << p.second.first <<'\t'<< pnt.x<<' '<<pnt.y<<std::endl;
	}



	of << endl << endl;

}


INIT_SQLBUILDER


auto bind(SQLBuilder  & sb, int const index, int const value)
{
//	ASSERT(handle);
	sqlite3_stmt *ppStmt;
	std::string sql = "insert into sweep(id, angle, distance) VALUES(?1,?2,?3)";

	auto result = sqlite3_prepare_v2(sb.gdb,
		sql.c_str(),
		sql.length(),
		&ppStmt,
		nullptr);

//	auto const result = sqlite3_bind_int(0,
	//	value);

	if (SQLITE_OK != result)
	{
	/*	throw sql_exception
		{
		  result,
		  sqlite3_errmsg(sqlite3_db_handle(handle.get()))
		};*/
	}
}

void RplidarReadingQueue::savePresentScan(int id, std::string & database, rp::RplidarProxy::ScanVecType * theScan,
	float tilt)
{

	//sqlite3_stmt *ppStmt;
	//std::string sql = "insert into sweep(id, angle, distance) VALUES(?1,?2,?3)";



	//rp::RplidarProxy::ScanVecType * sv;

	SQLBuilder  sb;
	sb.createOrOpenDatabase(database);

	//auto result = sqlite3_prepare_v2(sb.gdb,
	//	sql.c_str(),
	//	sql.length(),
	//	&ppStmt,
	//	nullptr);
	
	if (theScan == nullptr) { //possibly not spooled up yet?
		SGUP_ODSA(__FUNCTION__, __LINE__, "bad scan nullptr");
		return;
	}


	if (theScan->size() == 0) {
		SGUP_ODSA(__FUNCTION__, __LINE__, "0 size scan");
		return;
	}
	else
	{
		SGUP_ODSA(__FUNCTION__, "size:", theScan->size());
	}

	//else if (theScan->size() == 0)
	//{
	//	SGUP_ODSA(__FUNCTION__, __LINE__, "getScan size 0");
	//	return;
	//}
	//else
	//	SGUP_ODSA(__FUNCTION__, __LINE__, "getScan size:", sv->size());
	


	std::stringstream ss;
	ss<< "insert into sweep(id, angle, distance) VALUES \n";
	
	//insert all this in SQL
	//if (sv != nullptr);
		for (auto &reading : *theScan) {

			auto angle = reading.first;
			auto dist = reading.second.first;

			auto s = boost::format("(%1%,%2%,%3%),") % id %angle %dist;

	//		SGUP_ODSA(__FUNCTION__,"looping...", s);
			ss << s << std::endl;

		/*	sqlite3_bind_double( ppStmt,1, id);
			sqlite3_bind_double(ppStmt, 2, angle);
			sqlite3_bind_double(ppStmt, 3, dist);
			
			rc = sqlite3_step(ppStmt)) == SQLITE_ROW*/

		} 

		ss.seekp(-2, ss.cur); 
		ss << "; ";
	//	REPLACE_LAST_CHAR_ON_SS(ss, ';')

		SGUP_ODSA(__FUNCTION__, ss.str());


		auto b=sb.sendSQL(ss.str());

		if (!b)
		{
			SGUP_ODSA(__FUNCTION__, __LINE__, "sendsql failed");
		}

	//end sql

	


}


bool RplidarReadingQueue::sendSQL(std::string & sql)
{
	//TODO:this sql stuff
	return false;
}

const char * rp::measure::debugPrint()
{
	sprintf(temp, "%s theta: %03.2f Dist: %08.2f Q: %d \n",
		(sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
		(angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
		distance_q2 / 4.0f,
		sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

	return temp;
}





