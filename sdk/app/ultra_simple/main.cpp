
#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "C:\usr\include\GregUtils\strguple.h"
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>

INIT_STRGUPLE

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

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
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

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}




struct RplidarReadingQueue {
	struct measure	: _rplidar_response_measurement_node_t {
	char temp[512];
	
	const char * debugPrint() {
		sprintf(temp, "%s theta: %03.2f Dist: %08.2f Q: %d \n",
			(sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
			(angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
			distance_q2 / 4.0f,
			sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

		return temp;
	}

	inline float distance() {
		return distance_q2 / 4.0f;
	}

	inline float theta() {
		return (angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
	}
};

	int fromRadial;
	int toRadial;
	bool keepGoing = true;
	bool useRangeFilter = false;
	_u32 baud;
	char *     opt_com_path;
	boost::thread * scanThread = nullptr;

	boost::circular_buffer<measure> * cb = nullptr;
	RplidarReadingQueue(float fromRadial, float toRadial, int qSize, _u32 baud = 256000, char * opt_com_path = "\\\\.\\com3") :fromRadial(fromRadial), toRadial(toRadial),baud(baud), opt_com_path(opt_com_path){
		cb = new boost::circular_buffer<measure>(qSize);
	}

	RplidarReadingQueue(int size) {
		cb = new boost::circular_buffer<measure>(size);
	}

	void setRange(int from, int to) {
		fromRadial = from;
		toRadial = to;
		useRangeFilter = true;
	}

	void get(measure & m, int fromRadial, int toRadial) {

	}

	inline void push(measure &m) {
		
		if (m.distance() == 0) return;

		auto thet = m.theta();

		if(useRangeFilter)
			if (thet >= fromRadial && thet <= toRadial) {
				cb->push_back(m);
			}
		else
			cb->push_back(m);
		
	}

	~RplidarReadingQueue() {
		if(cb)
			delete cb;
	}

	void stop()
	{
		scanThread->interrupt();
	}

	bool run() {
		
		_u32         baudrateArray[2] = { 115200, 256000 };
		_u32         opt_com_baudrate = 0;
		u_result     op_result;

		RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
		if (!drv) {
			fprintf(stderr, "insufficent memory, exit\n");
			return false;
		}

		drv->startMotor();
		// start scan...
		drv->startScan(0, 1);


		char temp[512];

		// fetech result and print it out...
		while (keepGoing) {
			rplidar_response_measurement_node_t nodes[8192];
			size_t   count = _countof(nodes);

			op_result = drv->grabScanData(nodes, count);

			if (IS_OK(op_result)) {
				drv->ascendScanData(nodes, count);
				for (int pos = 0; pos < (int)count; ++pos) {
		
					measure m = (measure&)nodes[pos];
					push(m);
					SGUP_ODS(m.debugPrint());


				}
			}
			try {
				boost::this_thread::interruption_point();
			}
			catch (...)
			{
				SGUP_ODS(__FUNCTION__, "thread interrupt");
				break;
			}
		}

		drv->stop();
		drv->stopMotor();
	}

	bool runThreaded() {
		scanThread=new boost::thread(std::bind(&RplidarReadingQueue::run, this));
		return true;
	}

};


int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;




	RplidarReadingQueue rp(320, 60, 1000);
	rp.runThreaded();

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
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
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
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
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
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
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
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}

