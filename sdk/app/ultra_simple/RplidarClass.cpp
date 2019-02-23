
#include "RplidarClass.h"

//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
//#include "C:\usr\include\GregUtils\strguple.h"
//#include <boost/circular_buffer.hpp>
//#include <boost/thread.hpp>
#include "windows.h"

INIT_RPLIDAR
INIT_STRGUPLE

RplidarReadingQueue::RplidarReadingQueue(float fromRadial, float toRadial, int qSize,
	_u32 baud /*= 256000*/, char * opt_com_path /*= "\\\\.\\com3"*/) :fromRadial(fromRadial), toRadial(toRadial), baud(baud), opt_com_path(opt_com_path), useRangeFilter(true)
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
	boost::mutex::scoped_lock lock(qMutex);
	return -1;
}

int RplidarReadingQueue::getFront(rp::measure & m)
{
	boost::mutex::scoped_lock lock(qMutex);

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

bool RplidarReadingQueue::push(rp::measure &m)
{
	boost::mutex::scoped_lock lock(qMutex);

	if (m.distance() == 0) return false;

	auto thet = m.theta();

	if (useRangeFilter)
		if (isInRange(thet)) {
			cb->push_back(m);
		//	SGUP_ODS("PUSH THETA:", thet);
			return true;
		}
		else {
			//		SGUP_ODS("REJECT THETA:", thet);
			return false;
		}
	else
		cb->push_back(m);

	return true;
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
		try {
			boost::this_thread::interruption_point();
		}
		catch (...)
		{
			SGUP_ODS(__FUNCTION__, "thread interrupt");

			SGUP_ODS("Stopping scan and motor");

			drv->stop();
			drv->stopMotor();

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

const char * rp::measure::debugPrint()
{
	sprintf(temp, "%s theta: %03.2f Dist: %08.2f Q: %d \n",
		(sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
		(angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
		distance_q2 / 4.0f,
		sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

	return temp;
}





