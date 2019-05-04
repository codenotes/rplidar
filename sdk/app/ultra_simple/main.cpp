
#include <stdio.h>
#include <stdlib.h>
//#include <algorithm>
//#include <iostream>
//#include <numeric>
//#include <vector>
#include <future>
#include "RplidarClass.h"

//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
//#include "C:\usr\include\GregUtils\strguple.h"
//#include <boost/circular_buffer.hpp>
//#include <boost/thread.hpp>

#include "\usr\include\GregUtils\ansi_utils.h"


#define DB_PATH R"(C:\usr\data\cirrus.db)"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif
//using rp::standalone::rplidar;

#if 0
bool checkRPLIDARHealth(rp::standalone::rplidar::RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}
#endif
#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

template <typename T>
std::vector<T> GetLinearFit(std::vector<T> xData , const std::vector<T>& data)
{
	T xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
	
	for (long i = 0; i < data.size(); i++)
	{
		xData.push_back(static_cast<T>(i));
	}
	for (long i = 0; i < data.size(); i++)
	{
		xSum += xData[i];
		ySum += data[i];
		xxSum += xData[i] * xData[i];
		xySum += xData[i] * data[i];
	}
	slope = (data.size() * xySum - xSum * ySum) / (data.size() * xxSum - xSum * xSum);
	intercept = (ySum - slope * xSum) / data.size();
	std::vector<T> res;
	res.push_back(slope);
	res.push_back(intercept);
	return res;
}



double slope(const std::vector<double>& x, const std::vector<double>& y) {
	const auto n = x.size();
	const auto s_x = std::accumulate(x.begin(), x.end(), 0.0);
	const auto s_y = std::accumulate(y.begin(), y.end(), 0.0);
	const auto s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
	const auto s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
	const auto a = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
	return a;
}

void testSlope() {
	std::vector<double> x{ 6, 5, 11, 7, 5, 4, 4 };
	std::vector<double> y{ 2, 3, 9, 1, 8, 7, 5 };
	std::cout << slope(x, y) << '\n';  // outputs 0.305556
}




#define INIT_RPLIDAR boost::thread * RplidarReadingQueue::scanThread=nullptr;

using namespace std;

//RplidarReadingQueue * grp = nullptr;




//void startLidar(float fromRadial, float toRadial, int qSize) {
//
//	if(grp!=nullptr)
//		grp=new RplidarReadingQueue( fromRadial,  toRadial,  qSize);
//	
//	grp->runThreaded();
////	rp.join();
//}

//void stopLidar(float fromRadial, float toRadial, int qSize) {
//
//	grp->stop();
//
//}


//using rp::standalone::rplidar;
template <typename T>
struct TypeParser {};

template <typename Ret, typename... Args>
struct TypeParser<Ret(Args...)> {
	static std::function<Ret(Args...)> createFunction(const FARPROC lpfnGetProcessID) {
		return std::function<Ret(Args...)>(reinterpret_cast<Ret(__stdcall *)(Args...)>(lpfnGetProcessID));
	}
};

template <typename T>
std::function<T> loadDllFunc(const std::string& funcName, HINSTANCE hGetProcIDDLL) {
	// Load DLL.


	// Locate function in DLL.
	FARPROC lpfnGetProcessID = GetProcAddress(hGetProcIDDLL, funcName.c_str());

	// Check if function was located.
	if (!lpfnGetProcessID) {
		//std::cerr << "Could not locate the function \"" << funcName << "\" in DLL\"" << dllName << "\"" << std::endl;
		SGUP_ODS(__FUNCTION__, "function could not be loaded or dll does not exist and could not be loaded");
		return NULL;
		//	exit(EXIT_FAILURE);
	}

	// Create function object from function pointer.
	return TypeParser<T>::createFunction(lpfnGetProcessID);
}


inline bool isKeyDown(int keyCode)
{
	return ((GetAsyncKeyState(keyCode) & 0x8000) ? 1 : 0);
};

inline bool isKeyUp(int keyCode)
{
	return ((GetAsyncKeyState(keyCode) & 0x8000) ? 0 : 1);
};


















INIT_RP_RPLIDAR_PROXY
INIT_STRGUPLE

#include <chrono>
#include <thread>
#include <optional>

//std::optional<std::string>  findRplidarComPort()
//{
//	rp::RplidarProxy::wmiReportType *temp;
//	rp::RplidarProxy::fnGetComPortLidar(&temp);
//
//	for (auto &i : *temp)
//	{
//		auto com = i.first.substr(i.first.find(":") + 1);
//		auto desc = i.second.substr(i.second.find(":") + 1);
//
//		if (desc.find("Silicon") != std::string::npos) {
//
//			remove_if(com.begin(), com.end(), isspace);
//			return com;
//			//cout << "FOUND RPLIDAR COM PORT:" << com << ", " << desc << endl;
//		}
//		else {
//			return std::nullopt;
//		//	cout << "Did not find a rplidar com port (Silicon Labs)" << endl;
//		}
//
//	}
//}
void deleter(std::string * s) { delete s; }


void recorder(int msec, const std::string & path) {

	using thing = rp::RplidarProxy::ScanVecType2 *;

	auto fn = [&](int sleepfor)->thing {

			   
		std::this_thread::sleep_for(std::chrono::milliseconds(sleepfor));
		return nullptr;
	};

	try {
		while (true) {
			std::future<thing> result_future = std::async(std::launch::deferred, fn, 500);
		}
	}
	catch (...)
	{
		cout << "exception thrown!" << endl;
	}


}


int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result; 

	ANSI_Util::EnableVTMode();



	std::string dllloc = R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Debug\rplidarReader.dll)";
	 
#ifdef _DEBUG
	HMODULE h=LoadLibraryA(R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Debug\rplidarReader.dll)");
#else
	HMODULE h = LoadLibraryA(R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Release\rplidarReader.dll)");
#endif

	
//	auto proc=GetProcAddress(h, "StartLidar");
//	auto test= loadDllFunc<rp::RplidarProxy::GetScanWithExpiryT>("GetScanWithExpiry", h); 
	//auto fnTest = std::function<int(void)>(proc);
	//auto fnTest = loadDllFunc<int(void)>( "test",h);

	//cout << fnTest() << endl;
	
	RP_INIT_DLL_FUNCTIONS(h);


	


	//test area
#ifdef TEST0
	SGUP_ODSA(__FUNCTION__);
	rp::RplidarProxy::wmiReportType *temp;

	try {

		rp::RplidarProxy::fnGetComPortLidar(&temp);
	}
	catch (std::exception &e)
	{
		SGUP_ODSA(__FUNCTION__, "exception getting ports", e.what());
		return 0;
	}


	for (auto &i : *temp)
	{
		auto com = i.first.substr(i.first.find(":") + 1);
		auto desc = i.second.substr(i.second.find(":") + 1);
		SGUP_ODSA(__FUNCTION__, desc);

		if (desc.find("Silicon") != std::string::npos) {

			SGUP_ODSA(__FUNCTION__, "found silicon, adding");
			//	remove_if(com.begin(), com.end(), isspace);
				//return com;
			cout << com << endl;
			//cout << "FOUND RPLIDAR COM PORT:" << com << ", " << desc << endl;
		}
		else {
			//	return std::nullopt;
				//	cout << "Did not find a rplidar com port (Silicon Labs)" << endl;
		}

	}
	
	return 0;
#endif

	//end test area

	using namespace std;

	rp::RplidarProxy::ScanVecType2 * scanPointer = nullptr;
	int scanID = 0;
	std::string path(DB_PATH);

	auto fnTestDB = [&]() {
		std::optional<int> num;
		std::stringstream ss;
		num = 16;

		int next = rp::RplidarProxy::fnGetScanFromDatabase(&scanPointer, std::string(DB_PATH), num,std::nullopt);
		rp::RplidarProxy::fnSVToString(scanPointer, ss);
		
		cout << ss.str();

		cout << endl <<"next:"<< next << endl;
	};

	/*std::string sql("select * from vectors;");

	std::shared_ptr<std::string> thestring(new std::string(sql), deleter);

	rp::RplidarProxy::fnSendSQL(std::string(DB_PATH), thestring);

	cout << *thestring << endl;
	return 0;*/
	//auto fnStart = loadDllFunc<int(float, float, int)>(dllloc.c_str(), "StartLidar",h);
	//auto fnGet = loadDllFunc<int(rp::measure&)>(dllloc.c_str(), "GetMeasure",h);
	//auto fnStop = loadDllFunc<int(void)>(dllloc.c_str(), "StopLidar",h);
	//auto fnGetLidarStatus = loadDllFunc<rp::enumLidarStatus(void)>(dllloc.c_str(), "GetLidarStatus", h);
	
	/*rp::RplidarProxy::fnStartLidar = loadDllFunc<rp::RplidarProxy::StartLidarT>("StartLidar", h);
	rp::RplidarProxy::fnGetMeasure = loadDllFunc<rp::RplidarProxy::GetMeasureT>("GetMeasure", h);
	rp::RplidarProxy::fnStopLidar = loadDllFunc<rp::RplidarProxy::StopLidarT>("StopLidar", h);
	rp::RplidarProxy::fnGetLidarStatus = loadDllFunc<rp::RplidarProxy::GetLidarStatusT>("GetLidarStatus", h);
	rp::RplidarProxy::fnStartLidarWithParams = loadDllFunc<rp::RplidarProxy::StartLidarWithParamsT>("StartLidarWithParams", h);*/

	auto fnInit = [&]() {

		auto res = rp::RplidarProxy::findRplidarComPort();

		if (!res) {
			cout << "didnt find a Silicon COM port, so rplidar not plugged in or something" << endl;
			return 1;
		}
		else
		{
			cout << "LIDAR found on:" << *res << endl;
		}

		//cout << "Loading cirrus.db:" << DB_PATH << endl;



		cout << "starting lidar, press escape to quit reading" << endl;


		rp::RplidarProxy::fnStartLidarWithParams(0, 0, 1000, 256000, (*res).c_str());
		rp::measure m;
		int cnt;

		using  rp::RplidarProxy;
		//std::vector< std::pair< float, float> >  theScan;

		cout << "waiting..." << endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(5000));

		cout << "done waiting..." << endl;


	};


	auto fnReader = [&]() {

		while (1) {
			//cnt = rp::RplidarProxy::fnGetMeasure(m);
		//	rp::RplidarProxy::fnGetScan(&scanPointer);

			rp::RplidarProxy::fnGetScanWithExpiry(&scanPointer, 1500);


			/*	for (auto &i : theScan) {
					cout << i.first << ":" << i.second << endl;
				}*/
			if (scanPointer == nullptr) continue;

			auto sz = scanPointer->size();

			cout << "scan..." << sz << endl;

			if (sz) {
				//rp::RplidarProxy::fnSavePresentScan(scanID++, std::string(DB_PATH), scanPointer);
				cout << "would do scan to database..." << endl;
				rp::RplidarProxy::fnSaveScanToDatabase(scanPointer, path, std::optional<int>(),std::nullopt);

			}


			//	rp::RplidarProxy::fnDumpScanToFile(std::string("c:\\temp\\outfile.txt"),scanPointer, true);
			/*	if (isKeyDown(VK_PAUSE)) {
					for (auto &i : *scanPointer) {
						cout << i.first << ":" << i.second << endl;
					}

					}*/




					//if(cnt!=-1)
					//	cout << "count:" << cnt << "\t " << m.debugPrint() << endl;

			if (isKeyDown(VK_ESCAPE)) {
				cout << "escape pressed, shutting down..." << endl;

				//for (auto &i : *scanPointer) {
				////	auto p = rp::measure::convertAngleDist(i.first, i.second);
				//	cout <<i.first<<"->"<<  i.second << endl;
				//	//roundf(p.first * 10) / 10
				//}

				break;
			}

			rp::RplidarProxy::fnDestroyScan(&scanPointer);
			//print out everything






			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}

	};

	auto fnDBReader = [&](std::optional< std::pair<int, int> > opt = std::nullopt, bool loop = false, bool rst = false, int delayMS = 500) {



		while (1) {
			
			auto scanRet=rp::RplidarProxy::fnGetRangeOfScansFromDatabase(&scanPointer, path, opt, loop, rst,std::nullopt);


			if (scanPointer == nullptr) {
				cout << "null scan, retval:" <<scanRet<< endl;
			}
			else
			{

				auto sz = scanPointer->size();

				cout << "db read size:" << sz <<" scanRet:"<<scanRet<< endl;

				if (sz) {

				cout << RED_DEF << scanPointer->begin()->second.debugID << RESET_DEF << endl;
					//for (const auto & r : *scanPointer) {
					//	cout << RED_DEF<< r.second.debugID << RESET_DEF<<", ";
					//}

				}
				else
				{
					cout << YELLOW_DEF << "size 0 so must be no scan at that ID" << RESET_DEF << endl;
				}

				cout << "--" << endl;

			
				rp::RplidarProxy::fnDestroyScan(&scanPointer);

			}

			std::this_thread::sleep_for(std::chrono::milliseconds(delayMS));
			if (isKeyDown(VK_ESCAPE)) {
				cout << "escape pressed, shutting down..." << endl;
				break;
			}
		}



	};


	//fnInit(); //starts lidar up
	//fnReader();

	//fnDBReader(std::pair<int, int>{0,0},true,false, 250);
	fnDBReader(std::nullopt, true, false, 250);


	cout << "Waiting for reported shutdown..." << endl;
	rp::RplidarProxy::fnStopLidar();
	   	


//	auto res=RplidarReadingQueue::GetLinearFit({ {1,10},{2,11},{3,10},{4,11} });
//	cout << res.first << " " << res.second << endl;

#if 0

	RplidarReadingQueue rp(355, 10, 1000);
	rp.runThreaded();
	rp.join();

	return 0;

    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: %s\n", RPLIDAR_SDK_VERSION);

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
	rp::standalone::rplidar::RPlidarDriver * drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);


	char temp[512];

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                //printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                //    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                //    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                //    nodes[pos].distance_q2/4.0f,
                //    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

		//		measure m = (measure&)nodes[pos];

				/*sprintf(temp, "%s theta: %03.2f Dist: %08.2f Q: %d \n",
					(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
					(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					nodes[pos].distance_q2 / 4.0f,
					nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/

//				SGUP_ODS(m.debugPrint());
				

            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
on_finished:
	rp::standalone::rplidar::RPlidarDriver::DisposeDriver(drv);
    drv = NULL;

#endif
    return 0;
}

