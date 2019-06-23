
#include <stdio.h>
#include <stdlib.h>
//#include <algorithm>
//#include <iostream>
//#include <numeric>
//#include <vector>
#include <future>
#include "RplidarClass.h"

#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 
#include "windows.h"
#include <boost/circular_buffer.hpp>

//#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
//#include "C:\usr\include\GregUtils\strguple.h"
//#include <boost/circular_buffer.hpp>

//#include <boost/thread.hpp>
//TRACEDESIGNTIME = true

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

	rp::RplidarProxy::areFunctionsInit = true; //if this is called even once, assume it has been called via the INIT macro and all function objects are loaded
	// Locate function in DLL.
	FARPROC lpfnGetProcessID = GetProcAddress(hGetProcIDDLL, funcName.c_str());

	// Check if function was located.
	if (!lpfnGetProcessID) {
		//std::cerr << "Could not locate the function \"" << funcName << "\" in DLL\"" << dllName << "\"" << std::endl;
		SGUP_ODS(__FUNCTION__, "function could not be loaded or dll does not exist and could not be loaded:", funcName.c_str());
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

#include <boost/lexical_cast.hpp>

using namespace boost::asio;
#define READ_SIZE 512


void cbSerial(std::vector<unsigned char> & buf  ) {

}



double a[3], w[3], Angle[3], T;
void DecodeIMUData(unsigned char chrTemp[])
{
	switch (chrTemp[1])
	{
	case 0x51:
		a[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 16;
		a[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 16;
		a[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 16;
		T = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		break;
	case 0x52:
		w[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 2000;
		w[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 2000;
		w[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 2000;
		T = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		break;
	case 0x53:
		Angle[0] = (short(chrTemp[3] << 8 | chrTemp[2])) / 32768.0 * 180;
		Angle[1] = (short(chrTemp[5] << 8 | chrTemp[4])) / 32768.0 * 180;
		Angle[2] = (short(chrTemp[7] << 8 | chrTemp[6])) / 32768.0 * 180;
		T = (short(chrTemp[9] << 8 | chrTemp[8])) / 340.0 + 36.25;
		printf("a = %4.3f\t%4.3f\t%4.3f\t\r\n", a[0], a[1], a[2]);
		printf("w = %4.3f\t%4.3f\t%4.3f\t\r\n", w[0], w[1], w[2]);
		printf("Angle = %4.2f\t%4.2f\t%4.2f\tT=%4.2f\r\n", Angle[0], Angle[1], Angle[2], T);
		break;
	}
}


#define QSIZE 50
class SPBoost {
	inline static boost::asio::io_service io;
	inline static boost::asio::serial_port * port=nullptr;// (io);
	inline static std::size_t bytesTransferred;
	inline static std::array<unsigned char, READ_SIZE> bytes;
	inline static boost::circular_buffer<unsigned char> cb{ 512 };
	using thing = std::function<void(void)>;
	using uchar = unsigned char;
public:
	
	struct AngVelocity {
		uchar wxL;
		uchar wxH;
		uchar wyL;
		uchar wyH;
		uchar wzL;
		uchar wzH;
		uchar TL;
		uchar TH;
		uchar Sum;
		double wx;
		double wy;
		double wz;
		double temp;
	};

	struct Angles {

		uchar RollL;
		uchar RollH;
		uchar PitchL;
		uchar PitchH;
		uchar YawL;
		uchar YawH;
		uchar TL;
		uchar TH;
		uchar Sum;
		double roll;
		double pitch;
		double yaw;
		double temp;


	};


	struct Acceleration {
			uchar AxL;
			uchar AxH; 
			uchar AyL; 
			uchar AyH; 
			uchar AzL; 
			uchar AzH; 
			uchar TL; 
			uchar TH; 
			uchar Sum;
			double ax;
			double ay;
			double az;
			double temp;
	};


	inline static boost::circular_buffer<Angles> cbAngles{ QSIZE };
	inline static boost::circular_buffer<Acceleration> cbAccel{ QSIZE };
	inline static boost::circular_buffer<AngVelocity> cbAngleVel{ QSIZE };

	static void interpretReadings() {

		if (cb.empty() || (cb.size()<100)) return;


		auto fnAngVelocity = [&]() {
			//we need four additional
			AngVelocity av;
			av.wxL = cb.front(); cb.pop_front();//2
			av.wxH = cb.front(); cb.pop_front();//3
			av.wyL = cb.front(); cb.pop_front();//4
			av.wyH = cb.front(); cb.pop_front();//5
			av.wzL = cb.front(); cb.pop_front();//6
			av.wzH = cb.front(); cb.pop_front();//7
			av.TL =  cb.front() ; cb.pop_front();//8
			av.TH =  cb.front() ; cb.pop_front();//9
			av.Sum = cb.front(); cb.pop_front();//10

			av.wx = (short(av.wxH << 8 | av.wxL)) / 32768.0 * 2000;
			av.wy = (short(av.wyH << 8 | av.wyL)) / 32768.0 * 2000;
			av.wz = (short(av.wzH << 8 | av.wzL)) / 32768.0 * 2000;
			av.temp = (short(av.TH << 8 | av.TL)) / 340.0 + 36.25;
			
			cbAngleVel.push_back(av);
		
		};


		auto fnAngles = [&]() {
			
			Angles ang;

			ang.RollL= cb.front(); cb.pop_front();//2
			ang.RollH= cb.front(); cb.pop_front();//3
			ang.PitchL= cb.front(); cb.pop_front();//4
			ang.PitchH= cb.front(); cb.pop_front();//5
			ang.YawL= cb.front(); cb.pop_front();//6
			ang.YawH= cb.front(); cb.pop_front();//7
			ang.TL= cb.front(); cb.pop_front();//8
			ang.TH= cb.front(); cb.pop_front();//9
			ang.Sum= cb.front(); cb.pop_front();//10

			
			ang.roll = (short(ang.RollH << 8 | ang.RollL)) / 32768.0 * 180;
			ang.pitch = (short(ang.PitchH << 8 | ang.PitchL)) / 32768.0 * 180;
			ang.roll = (short(ang.YawH << 8 | ang.YawL)) / 32768.0 * 180;
			ang.temp = (short(ang.TH << 8 | ang.TL)) / 340.0 + 36.25;
			
			cbAngles.push_back(ang);
		};


		auto fnAccel =[&]() {
			
			Acceleration ac;

			ac.AxL = cb.front(); cb.pop_front();//2
			ac.AxH = cb.front(); cb.pop_front();//3
			ac.AyL = cb.front(); cb.pop_front();//4
			ac.AyH = cb.front(); cb.pop_front();//5
			ac.AzL = cb.front(); cb.pop_front();//6
			ac.AzH = cb.front(); cb.pop_front();//7
			ac.TL = cb.front(); cb.pop_front();//8
			ac.TH = cb.front(); cb.pop_front();//9
			ac.Sum = cb.front(); cb.pop_front();//10

			ac.ax = (short(ac.AxH << 8 | ac.AxL)) / 32768.0 * 16;
			ac.ay = (short(ac.AyH << 8 | ac.AyL)) / 32768.0 * 16;
			ac.az = (short(ac.AzH << 8 | ac.AzL)) / 32768.0 * 16;
			ac.temp = (short(ac.TH << 8 | ac.TL)) / 340.0 + 36.25;
			
			cbAccel.push_back(ac);

		};


		auto fnEatTillHeader = [&](bool printit=false) {
			int i = 0;
			
			if(printit)
				printf(GREEN_DEF "before while:\t\t%x %x %x %x %x %x %x %x %x %x | %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10], cb[11], cb[12]);

			
			while (cb[0] != 0x55) {
				//SG2("NOT header, cb.size:", cb.size())
				//_RPT2(0, "not 55, pop:%x,%x", cb[0], cb.front());
			//	SGUP_ODSA("dump:",(int)cb[0], (int)cb[1], (int)cb[2], (int)cb[3], (int)cb[4], (int)cb[5], (int)cb[6], (int)cb[7], (int)cb[8], (int)cb[9]);
				//printf("about pop (0-9):%x %x %x %x %x %x %x %x %x\n", cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
				cb.pop_front(); //throw away, remaining 10
				if(printit)
					printf("just popped (0-9):"  GREEN_DEF "%x " RESET_DEF "%x %x %x %x %x %x %x %x\n", cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
				i++;
				//	if (cb.empty()) { return; }
				//	ak;

			}

			return i;


		};

		std::map<uchar, thing> typeSwitcher = { {0x52,fnAngVelocity},{0x53,fnAngles},{0x51,fnAccel} };

	//	cout << "!" << endl;

		printf(AQUABAR_BOLDWHITE_DEF "START:\t%x %x %x %x %x %x %x %x %x %x %x %x\n" RESET_DEF,  cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10],cb[11]);


		if (cb[0] != 0x55)//if we just started or we don't have the begging of frame at head of q eat until we do.  This should execute rarely.
			fnEatTillHeader(true);
		else
			printf(INVERSEYELLOW_DEF "HEADER SET CORRECTLY FROM PREVIOUS CALL!\n" RESET_DEF);

		
		cb.pop_front(); //get rid of the hessage header now, 0x55...

	//	SG2("found header");
		if (cb.size() < 10) { cout << RED_DEF << "oh no, <10" << RESET_DEF << endl; return; } //in case we had a bunch of inbetween data and a header wasn't found

		//printf(GREEN_DEF "data:\t\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);

		
		//what type of message is this? 
		uchar typ = cb.front();
	//	_RPT3(0, "type:%x,%x, typ:%x", cb[0], cb.front(), cb[0], typ);

		cb.pop_front(); //get rid of typ descripter, **9** left before next header, which we want to leave in front of the cb

		if (typeSwitcher.find(typ) != typeSwitcher.end()) {
			//printf(YELLOW_DEF "%x\n" RESET_DEF, typ);
			printf(YELLOW_DEF "%x:TYPE SWITCH:\t%x %x %x %x %x %x %x %x %x %x || %x\n" RESET_DEF,typ, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10]);

			auto n = fnEatTillHeader(false);
			printf(RED_DEF"%d\n" RESET_DEF,n);
			
			printf(INVERSEMAJ_DEF "PASS ON:\t\t%x %x %x %x %x %x %x %x %x %x || %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10], cb[11]);
			
		//	typeSwitcher[typ]();
		}
		else
		{
			printf(RED_DEF "message?:%x\n" RESET_DEF, typ);
		}
		//	_RPT1(0, "message?:%x", typ);
			


	}


	static void read_callback(const boost::system::error_code &error,	std::size_t  bytes_transferred)	{

		if (!error) {
			
			//port->async_read_some(buffer(bytes),
			//	boost::bind( SPBoost::read_callback,  boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)//boost::ref(bytesTransferred))
			//	);
			port->async_read_some(buffer(bytes), read_callback);
			//cout << "reading:" << bytes_transferred << endl;
			//cb.push_back(bytes.begin(), bytes.end());
			

			printf(BLUEBAR_BOLDWHITE_DEF "%d %d:before inserted:\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, bytes_transferred, this_thread::get_id(), cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
			//cb.insert(cb.begin(), bytes.begin(), bytes.end());
			//std::copy(begin(bytes), end(bytes), std::front_inserter(cb));
			for (int i = 0; i < bytes_transferred; i++)
			{
				cb.push_back(bytes[i]);
			}

			printf(BLUEBAR_BOLDWHITE_DEF "after inserted:\t\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);


			interpretReadings();

			
		}
		else {
			cout << "error?" << endl;
		}

		//if (error || !bytes_transferred)
		//{
		//	// No data was read!
		//	data_available = false;
		//	return;
		//}

	//	timeout.cancel();  // will cause wait_callback to fire with an error
		//data_available = true;
	}

public:
	SPBoost(std::string com, int bufSize = READ_SIZE) {
		using namespace boost;

		//asio::io_service io;
		//asio::serial_port port(io);
		port = new asio::serial_port(io);


		port->open(com);
		port->set_option(asio::serial_port_base::baud_rate(115200));

		/*unsigned char c[255];*/

		// Read 1 character into c, this will block
		// forever if no character arrives.

		port->async_read_some(buffer(bytes), read_callback);
		//asio::read(port, asio::buffer(&c, bufSize));
		io.run();
		cout << "constructor done" << endl;
		//port.close();

		//return c;
	}
};




int main(int argc, const char * argv[]) {


	//ANSI_Util::EnableVTMode();
	//auto x = ((unsigned char)0x55 == 'U') ? 1 : 0;
//	cout << x << endl;
	//return 0;

	cout << "starting..." << endl;
	SPBoost sp("COM6", READ_SIZE);


	return 0;


#ifdef _DEBUG
	HMODULE h = LoadLibraryA(R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Debug\rplidarReader.dll)");
#else
	HMODULE h = LoadLibraryA(R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Release\rplidarReader.dll)");
#endif
	RP_INIT_DLL_FUNCTIONS(h);

	rp::ROSArgs args;
	std::string s;
	bool b;

	//args["spinMsec"] = "2"; //Hz
	//#ROS_INIT_SWITCH to tell what this stuff does

	rp::RplidarProxy::fnROSAction(args,  rp::enumROSCommand::INTITIALIZE);
	args.clear();

	
	b = rp::RplidarProxy::fnROSAction(args, rp::enumROSCommand::START_MOTOR);


	args["topic"] = "/rplidarScan";

	 b=rp::RplidarProxy::fnROSAction(args, rp::enumROSCommand::START_SUB);
	SGUP_DEBUGLINE
	if (!b) {
		cout << "oh no, topic isn't found or something else wrong" << endl;
		return 0;

	}

	rp::RplidarProxy::ScanVecType2 * sv;
	
	//rp::RplidarProxy::fnROSAction(args, rp::enumROSCommand::STOP_MOTOR);
	//cin >> s;
	//return 0;

	cout << RED_DEF <<"entering loop"<< RESET_DEF << endl;

	while(1)
		 {

		if (isKeyDown(VK_ESCAPE)) break;

			rp::RplidarProxy::fnGetROSScan(&sv);

			if (sv) {

			//	cout << "got something:" << sv->size() << endl;
				for (auto &[deg, beam] : *sv) {
					
				
					cout << GREEN_DEF << deg << ":" << YELLOW_DEF << ":" << beam.distance << "\t"<<RESET_DEF;
				}
				cout << endl;
				//do something

				rp::RplidarProxy::fnDestroyScan(&sv);
			}
			else
			{
			}
			
		}

	rp::RplidarProxy::fnROSAction(args, rp::enumROSCommand::STOP_MOTOR);
	rp::RplidarProxy::fnROSAction(args, rp::enumROSCommand::SHUTDOWN);
	cout << endl << "exiting...any key" << endl;
	
	cin >> s;
	return 0;
}

#if 0
int main2(int argc, const char * argv[]) {
	const char * opt_com_path = NULL;
	_u32         baudrateArray[2] = { 115200, 256000 };
	_u32         opt_com_baudrate = 0;
	u_result     op_result;

	ANSI_Util::EnableVTMode();




	std::string dllloc =   R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Debug\rplidarReader.dll)";
	 
#ifdef _DEBUG
	HMODULE h=  LoadLibraryA(R"(C:\repos\lidar\rplidar_sdk\sdk\workspaces\vc10\x64\Debug\rplidarReader.dll)");
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


	auto fnReaderAndSaver = [&](const std::optional<std::string> & tableName) {

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
				rp::RplidarProxy::fnSaveScanToDatabase(scanPointer, path, std::optional<int>(), tableName);

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

	auto fnDBReader = [&](std::optional< std::pair<int, int> > opt = std::nullopt, bool loop = false, bool rst = false, int delayMS=500 ,
		const std::optional<std::string> & tableName= std::optional<std::string>(std::string("sweep"))) {



		while (1) {
			
			//auto scanRet=rp::RplidarProxy::fnGetRangeOfScansFromDatabase(&scanPointer, path, opt, loop, rst,std::nullopt);
			auto scanRet = rp::RplidarProxy::fnGetRangeOfScansFromDatabase(&scanPointer, path, opt, loop, rst, tableName);


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


	fnInit(); //starts lidar up


	//fnReader();

	//fnDBReader(std::pair<int, int>{0,0},true,false, 250);
	//fnDBReader(std::nullopt, true, false, 250, std::string("sweep"));

	fnReaderAndSaver("sweep1");

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
#endif
