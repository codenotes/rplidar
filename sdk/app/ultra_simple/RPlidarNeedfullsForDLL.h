
#pragma once

typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )


namespace rp {


	struct point {
		float x;
		float y;
		float z;
	};

	enum enumLidarStatus { STARTED, STOPPED, UNKNOWN };

	struct measure { // : _rplidar_response_measurement_node_t {

		__pragma(pack(push, 1))
			_u8    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
		_u16   angle_q6_checkbit; // check_bit:1;angle_q6:15;
		_u16   distance_q2;
		__pragma(pack(pop))

			char temp[512];

		const char * debugPrint();

		float distance();

		long double deg2rad(long double deg);

		long double rad2deg(long double rad);

		point convToCart(float r, float theta, float omega = 90.0f);

		float theta();
	};
}