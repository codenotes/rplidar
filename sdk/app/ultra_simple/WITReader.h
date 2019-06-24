#pragma once

#define READ_SIZE 512
#define QSIZE 50

class SPBoost {
	inline static boost::asio::io_service io;
	inline static boost::asio::serial_port * port = nullptr;// (io);
	inline static std::size_t bytesTransferred;
	inline static std::array<unsigned char, READ_SIZE> bytes;
	inline static boost::circular_buffer<unsigned char> cb{ 1024 };
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


	inline static boost::circular_buffer<Angles> circbAngles{ QSIZE };
	inline static boost::circular_buffer<Acceleration> circbAccel{ QSIZE };
	inline static boost::circular_buffer<AngVelocity> circbAngleVel{ QSIZE };

	inline static std::function< void(AngVelocity &) > cbAngelVel;
	inline static std::function< void(Angles &) > cbAngles;
	inline static std::function< void(Acceleration&) > cbAcceleration;




	static void interpretReadings() {

		if (cb.empty() || (cb.size() < 100)) return;


		auto fnAngVelocity = [&]() {
			//we need four additional
			AngVelocity av;
			av.wxL = cb.front(); cb.pop_front();//2
			av.wxH = cb.front(); cb.pop_front();//3
			av.wyL = cb.front(); cb.pop_front();//4
			av.wyH = cb.front(); cb.pop_front();//5
			av.wzL = cb.front(); cb.pop_front();//6
			av.wzH = cb.front(); cb.pop_front();//7
			av.TL = cb.front(); cb.pop_front();//8
			av.TH = cb.front(); cb.pop_front();//9
			av.Sum = cb.front(); cb.pop_front();//10

			av.wx = (short(av.wxH << 8 | av.wxL)) / 32768.0 * 2000;
			av.wy = (short(av.wyH << 8 | av.wyL)) / 32768.0 * 2000;
			av.wz = (short(av.wzH << 8 | av.wzL)) / 32768.0 * 2000;
			av.temp = (short(av.TH << 8 | av.TL)) / 340.0 + 36.25;

			circbAngleVel.push_back(av);

			if (cbAngelVel) cbAngelVel(av);

		};


		auto fnAngles = [&]() {

			Angles ang;

			ang.RollL = cb.front(); cb.pop_front();//2
			ang.RollH = cb.front(); cb.pop_front();//3
			ang.PitchL = cb.front(); cb.pop_front();//4
			ang.PitchH = cb.front(); cb.pop_front();//5
			ang.YawL = cb.front(); cb.pop_front();//6
			ang.YawH = cb.front(); cb.pop_front();//7
			ang.TL = cb.front(); cb.pop_front();//8
			ang.TH = cb.front(); cb.pop_front();//9
			ang.Sum = cb.front(); cb.pop_front();//10


			ang.roll = (short(ang.RollH << 8 | ang.RollL)) / 32768.0 * 180;
			ang.pitch = (short(ang.PitchH << 8 | ang.PitchL)) / 32768.0 * 180;
			ang.yaw = (short(ang.YawH << 8 | ang.YawL)) / 32768.0 * 180;
			ang.temp = (short(ang.TH << 8 | ang.TL)) / 340.0 + 36.25;

			circbAngles.push_back(ang);

			if (cbAngles) cbAngles(ang);
		};


		auto fnAccel = [&]() {

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

			circbAccel.push_back(ac);
			if (cbAcceleration) cbAcceleration(ac);

		};

		using namespace std;

		auto fnEatTillHeader = [&](bool printit = false) {
			int i = 0;

			if (printit)
				printf(GREEN_DEF "before while:\t\t%x %x %x %x %x %x %x %x %x %x | %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10], cb[11], cb[12]);


			while (cb[0] != 0x55) {
				//SG2("NOT header, cb.size:", cb.size())
				//_RPT2(0, "not 55, pop:%x,%x", cb[0], cb.front());
			//	SGUP_ODSA("dump:",(int)cb[0], (int)cb[1], (int)cb[2], (int)cb[3], (int)cb[4], (int)cb[5], (int)cb[6], (int)cb[7], (int)cb[8], (int)cb[9]);
				//printf("about pop (0-9):%x %x %x %x %x %x %x %x %x\n", cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
				cb.pop_front(); //throw away, remaining 10
				if (printit)
					printf("just popped (0-9):"  GREEN_DEF "%x " RESET_DEF "%x %x %x %x %x %x %x %x\n", cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
				i++;
				//	if (cb.empty()) { return; }
				//	ak;

			}

			return i;


		};

		std::map<uchar, thing> typeSwitcher = { {0x52,fnAngVelocity},{0x53,fnAngles},{0x51,fnAccel} };

		//	cout << "!" << endl;
	again:

		if (cb.size() <= 11) {
#ifdef DEBUG_WIT
			cout << RED_DEF << "oh no, <10,lets let the q refill are are ahead now" << RESET_DEF << endl;
#endif
			return;
		}
#ifdef DEBUG_WIT
		printf(AQUABAR_BOLDWHITE_DEF "%x|START:\t%x %x %x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, this_thread::get_id(), cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10], cb[11]);
#endif

		if (cb[0] != 0x55)//if we just started or we don't have the begging of frame at head of q eat until we do.  This should execute rarely.
			fnEatTillHeader(false);
		else {
#ifdef DEBUG_WIT
			printf(INVERSEYELLOW_DEF "HEADER SET CORRECTLY FROM PREVIOUS CALL!\n" RESET_DEF);
#endif
		}


		cb.pop_front(); //get rid of the hessage header now, 0x55...

	//	SG2("found header");
		//if (cb.size() < 10) { cout << RED_DEF << "oh no, <10" << RESET_DEF << endl; return; } //in case we had a bunch of inbetween data and a header wasn't found

		//printf(GREEN_DEF "data:\t\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);


		//what type of message is this? 
		uchar typ = cb.front();
		//	_RPT3(0, "type:%x,%x, typ:%x", cb[0], cb.front(), cb[0], typ);

		cb.pop_front(); //get rid of typ descripter, **9** left before next header, which we want to leave in front of the cb

		if (typeSwitcher.find(typ) != typeSwitcher.end()) {
			//printf(YELLOW_DEF "%x\n" RESET_DEF, typ);
#ifdef DEBUG_WIT
			printf(YELLOW_DEF "%x:TYPE SWITCH:\t%x %x %x %x %x %x %x %x %x %x || %x\n" RESET_DEF, typ, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10]);
#endif
			typeSwitcher[typ]();
			//auto n = fnEatTillHeader(false);
#ifdef DEBUG_WIT
			printf(RED_DEF"%d\n" RESET_DEF, n);
			printf(INVERSEMAJ_DEF "PASS ON:\t\t%x %x %x %x %x %x %x %x %x %x || %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9], cb[10], cb[11]);
#endif

		}
		else
		{
#ifdef DEBUG_WIT
			printf(RED_DEF "message?:%x\n" RESET_DEF, typ);
#endif
		}
		//	_RPT1(0, "message?:%x", typ);

		goto again;

	}


	static void read_callback(const boost::system::error_code &error, std::size_t  bytes_transferred) {
		using namespace boost::asio;
		using namespace std;

		if (!error) {

			//port->async_read_some(buffer(bytes),
			//	boost::bind( SPBoost::read_callback,  boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)//boost::ref(bytesTransferred))
			//	);
			port->async_read_some(buffer(bytes), read_callback);
			//cout << "reading:" << bytes_transferred << endl;
			//cb.push_back(bytes.begin(), bytes.end());

#ifdef DEBUG_WIT
			printf(BLUEBAR_BOLDWHITE_DEF "TX:%d |%x|:before inserted:\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, bytes_transferred, this_thread::get_id(), cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
#endif
			//cb.insert(cb.begin(), bytes.begin(), bytes.end());
			//std::copy(begin(bytes), end(bytes), std::front_inserter(cb));
			for (int i = 0; i < bytes_transferred; i++)
			{
				//printf("%x ", bytes[i]);
				cb.push_back(bytes[i]);
			}
			cout << endl;
#ifdef DEBUG_WIT
			printf(BLUEBAR_BOLDWHITE_DEF "after inserted:\t\t%x %x %x %x %x %x %x %x %x %x\n" RESET_DEF, cb[0], cb[1], cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]);
#endif


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
		using namespace std;

		//asio::io_service io;
		//asio::serial_port port(io);
		port = new asio::serial_port(io);


		port->open(com);
		port->set_option(asio::serial_port_base::baud_rate(115200));

		/*unsigned char c[255];*/

		// Read 1 character into c, this will block
		// forever if no character arrives.

		
	}

	void run() {
		using namespace std;
		port->async_read_some(boost::asio::buffer(bytes), read_callback);
		//asio::read(port, asio::buffer(&c, bufSize));
		io.run();
		cout << "constructor done" << endl;
		//port.close();

		//return c;
	}
};


