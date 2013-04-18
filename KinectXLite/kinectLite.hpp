/**
 * @file
 * @author  Manuel Martinez <manel@ira.uka.de>
 *
 */

#pragma once

#include <libusb-1.0/libusb.h>

#include <thread>
#include <mutex>

#include <vector>
#include <queue>
#include <cstring>
#include <cmath>


class KinectLite {
private:

	libusb_context *context;
	libusb_device_handle *camera, *motor;

	bool isoPkRecv;
	std::vector<libusb_transfer *> libusbTransfers;


	struct KinectTransfer {

		uint8_t data[2][2000000];
		uint8_t nData;
		uint8_t seq;
		uint8_t sync;
		int size;
		uint16_t half;

		void (*cb)(void *, uint8_t *, int);
		void *cbd;
		std::queue<std::string> q;
		bool enabled;

		KinectTransfer()
			: nData(),
			seq(),
			sync(),
			size(),
			half(),
			cb(NULL),
			cbd(NULL),
			q(),
			enabled(false)
		{}

		KinectTransfer(const KinectLite::KinectTransfer&) = delete;
		KinectTransfer operator=(const KinectLite::KinectTransfer&) = delete;
	};

	KinectTransfer iInfo, dInfo;

	bool isGood;
	std::mutex mtx;
	std::thread t;
	typedef std::lock_guard<std::mutex> Lock;

	static void MSLEEP( int n = 1) { std::this_thread::sleep_for( std::chrono::milliseconds(n) ); }
	static void NSLEEP( int n = 1) { std::this_thread::sleep_for( std::chrono:: nanoseconds(n) ); }


	void run() {

		while (isGood and context==NULL) MSLEEP(1);
		timeval tv; tv.tv_sec = 0; tv.tv_usec = 1;
		while (isGood) {

			NSLEEP(500);
			do {
				isoPkRecv = false;
				libusb_handle_events_timeout(context, &tv);
			} while (isoPkRecv==true);
		}
	};

public:
	int readSensorReg(int reg, int addr) {

		uint8_t data[512];
		uint8_t idata[] = { 'G', 'M', 3, 0, uint8_t(addr&255), uint8_t((addr>>8)&255), uint8_t(0&255), uint8_t((0>>8)&255), 1, 0, uint8_t(reg&255), uint8_t(0x80|((reg>>8)&255)), 0, 0 };
//		uint8_t idata[] = { 'G', 'M', 2, 0, uint8_t(addr&255), uint8_t((addr>>8)&255), uint8_t(0&255), uint8_t((0>>8)&255), uint8_t(reg&255), uint8_t(0x80|((reg>>8)&255)), 0, 0 };
		for (uint i=0; i<sizeof(idata); i++) data[i]=idata[i];

		//for (uint i=0; i<sizeof(idata); i+=2) printf("%02X%02X ", data[i+1], data[i]); printf("\n");

		libusb_control_transfer( camera, 0x40, 0, 0, 0, data, sizeof(idata), 0 );

		for (uint i=0; i<sizeof(idata); i++) data[i]=0;


		for (int tries = 100; data[0]!=0x52 or data[1]!=0x42; tries--) {
			MSLEEP(1);
			libusb_control_transfer( camera, 0xc0, 0, 0, 0,  data, sizeof(data), 0 );
			if (tries==1) return 0;
		}
//		for (int i=0; i<2; i+=2) printf("%02X%02X ", data[i+1], data[i]); printf(" OK\n");

//		libusb_control_transfer( camera, 0xc0, 0, 0, 0,  &data[2], 6, 0 );
//		libusb_control_transfer( camera, 0xc0, 0, 0, 0,  &data[8], 2*(256*data[3]+data[2]), 0 );

		int mgsSize = 8 + 2*(256*data[3]+data[2]);
		if (mgsSize>10) {for (int i=0; i<mgsSize; i+=2) printf("%02X%02X ", data[i+1], data[i]); printf("\n");}
		return 256*data[sizeof(data)-1]+data[sizeof(data)-2];
	}

	void writeReg(int reg, int val) {
		uint8_t data[] = { 'G', 'M', 2, 0, 3, 0, uint8_t(0&255), uint8_t((0>>8)&255), uint8_t(reg&255), uint8_t((reg>>8)&255), uint8_t(val&255), uint8_t((val>>8)&255) };
		libusb_control_transfer( camera, 0x40, 0, 0, 0, data, sizeof(data), 0 );
		do MSLEEP(1); while( !libusb_control_transfer( camera, 0xc0, 0, 0, 0,  data, sizeof(data), 0 ) );
	}

	int readReg(int reg) {

		uint8_t data[] = { 'G', 'M', 1, 0, 2, 0, uint8_t(0&255), uint8_t((0>>8)&255), uint8_t(reg&255), uint8_t((reg>>8)&255), 0, 0 };
		libusb_control_transfer( camera, 0x40, 0, 0, 0, data, 10, 0 );
		do MSLEEP(1); while( !libusb_control_transfer( camera, 0xc0, 0, 0, 0,  data, 12, 0 ) );
		return 256*data[11]+data[10];
	}

	static void isoCB(struct libusb_transfer *transfer) {


		KinectLite &kinect = *(KinectLite*)transfer->user_data;
		KinectTransfer &kt = transfer->endpoint==0x81?kinect.iInfo:kinect.dInfo;
		uint8_t *data = &kt.data[kt.nData][0];

		kinect.isoPkRecv = true;

		if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {

			kt.sync = false;
			if (transfer->status == LIBUSB_TRANSFER_NO_DEVICE) {
				kinect.isGood = false;
			}

			transfer->status = LIBUSB_TRANSFER_COMPLETED;
			if (kinect.isGood and kt.enabled) libusb_submit_transfer(transfer);
		}


		for (int i=0; i<transfer->num_iso_packets; i++) {


			uint8_t *buf = &((uint8_t*)transfer->buffer)[1920*i];
			int length = transfer->iso_packet_desc[i].actual_length;


			if (kt.half and length == kt.half) {

				memcpy( &data[kt.size], &buf[0], length );
				kt.size += length;
				kt.half = 0;
				continue;
			}

			if (length < 12) continue;

			int type = buf[3]&15;
			int hlength = (buf[6]<<8)+buf[7];

			++kt.seq;
			if ( buf[0] != 'R' or buf[1] != 'B' ) kt.sync = false;
			if ( buf[5] != kt.seq or length+kt.size-12 > 2000000) kt.sync = false;

			if ( type==1 ) {
				kt.sync=true;
				kt.seq =buf[5];
				kt.size=0;
				kt.half=0;
			}

			if ( type!=5 and length==960 and hlength>960)
				kt.half = hlength - 960;
			else if (length != hlength)
				kt.sync=false;

			if ( !kt.sync ) { continue; };

			memcpy( &data[kt.size], &buf[12], length-12 );
			kt.size += length-12;

			if ( type==5 ) {

				Lock l(kinect.mtx);
				if( kt.cb == NULL )
					kt.q.push(std::string((char *)data, kt.size));
				else
					(*kt.cb)(kt.cbd, data, kt.size);

				if( kt.q.size() > 2 ) kt.q.pop();
				kt.size=0;
				kt.sync=false;
				kt.nData = (kt.nData+1)%2;
			}
		}
		if (kinect.isGood and kt.enabled) libusb_submit_transfer(transfer);
	}

	int getRAW( KinectTransfer &kt, std::string &data, double timeOut=2. ) {
		do {
			if(!kt.q.empty()) {
				Lock l(mtx);
				data = kt.q.front();
				kt.q.pop();
				return true;
			}
			MSLEEP();
			timeOut -= 0.001;
		} while( isGood && timeOut>0. );
		return 0;
	}


	static void THELP(KinectLite *i) {i->run();};
public:
	KinectLite(int NTRANSFERS=16, int NPACKS=16) :
		context(NULL),
		camera(NULL),
		motor(NULL),
		isoPkRecv(),
		libusbTransfers(),
		iInfo(),
		dInfo(),
		isGood(true),
		mtx(),
		//t([&, this](){this->run();}) {
		t(THELP,this) {

		libusb_init( &context );

		libusb_device **devs;
		int cnt = libusb_get_device_list(context, &devs);
		for (int i =0; i<cnt; i++) {
			struct libusb_device_descriptor desc;
			libusb_get_device_descriptor(devs[i], &desc);

			if ( !camera && desc.idVendor == 0x045e && desc.idProduct == 0x02ae && !libusb_open(devs[i], &camera) )
				if( libusb_claim_interface (camera, 0) )
					{ libusb_close( camera ); camera = NULL; }

			if ( !motor  && desc.idVendor == 0x045e && desc.idProduct == 0x02b0 && !libusb_open(devs[i], &motor ) )
				if( libusb_claim_interface (motor, 0) )
					{ libusb_close( motor ); motor = NULL; }
		}
		libusb_free_device_list (devs, 1);

		isGood = camera;
		if (not isGood) return;

		iInfo.nData= dInfo.nData= 0;
		iInfo.size = dInfo.size = 0;
		iInfo.sync = dInfo.sync = false;
		iInfo.cb   = dInfo.cb   = NULL;
		iInfo.enabled = dInfo.enabled = false;



		for (int i=0; i<NTRANSFERS; i++) {
			libusbTransfers.push_back( libusb_alloc_transfer(NPACKS) );
			libusb_fill_iso_transfer( libusbTransfers.back(), camera, 0x81, new uint8_t[1920*NPACKS], 1920*NPACKS, NPACKS, isoCB, (void *)this, 0);
			libusb_set_iso_packet_lengths( libusbTransfers.back(), 1920);

			libusbTransfers.push_back( libusb_alloc_transfer(NPACKS) );
			libusb_fill_iso_transfer( libusbTransfers.back(), camera, 0x82, new uint8_t[1920*NPACKS], 1920*NPACKS, NPACKS, isoCB, (void *)this, 0);
			libusb_set_iso_packet_lengths( libusbTransfers.back(), 1920);
		}
	}

	~KinectLite() {

		isGood = false;
		t.join();

		for (uint8_t i=0; i<libusbTransfers.size(); i++) {
			delete[] libusbTransfers[i]->buffer;
			//libusb_free_transfer(libusbTransfers[i]); //TODO this breaks upon quitting, *might* cause memory leakage
		}

		if (camera) { libusb_release_interface( camera, 0); libusb_close( camera ); }
		if (motor) { libusb_release_interface( motor, 0); libusb_close( motor ); }

		libusb_exit( context );
	}

	KinectLite( const KinectLite &k);
	KinectLite &operator=( const KinectLite &k);

	bool isOpen() { return isGood; }

	double tilt() {

		if( !motor ) return 0.;

		uint8_t data[10];
		for(;;) {
			libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);
			if( data[9]==0 && data[8]!=128 ) break;
			MSLEEP(10);
		}

		return 0.5*int8_t(data[8]);
	}

	double tilt( double angle ) {

		if( !motor ) return 0.;

		libusb_control_transfer(motor, 0x40, 0x31,int(round(2*angle)), 0, NULL, 0, 0);

		uint8_t data[10];
		for(;;) {
			MSLEEP(10);
			libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);
			if( data[9]==0 ) break;
			if( data[9]==1 ) libusb_control_transfer(motor, 0x40, 0x31, 0, 0, NULL, 0, 0);
		}

		return tilt();
	}

	std::vector<double> acc() {

		if( !motor ) return std::vector<double>(3);

		uint8_t data[10];
		libusb_control_transfer(motor, 0xc0, 0x32, 0, 0, data, 10, 0);

		std::vector<double> r(3);

		r[0] = 9.81*(data[2]*256+data[3])/819.;
		r[1] = 9.81*(data[4]*256+data[5])/819.;
		r[2] = 9.81*(data[6]*256+data[7])/819.;

		return r;
	}

	void ledOff()    {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 0, 0, NULL, 0, 0); }
	void ledGreen()  {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 1, 0, NULL, 0, 0); }
	void ledRed()    {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 2, 0, NULL, 0, 0); }
	void ledYellow() {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 3, 0, NULL, 0, 0); }
	void ledBGreen() {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 4, 0, NULL, 0, 0); }
	void ledRedYel() {	if( motor ) libusb_control_transfer(motor, 0x40, 6, 6, 0, NULL, 0, 0); }

	void startDepth( bool hole=true, bool gmc=true, bool wBal=true ) {

		if ( not isGood ) return;

		if ( dInfo.enabled ) return; // Already enabled

		dInfo.enabled = true;
		//iInfo.enabled = false;

		for (uint32_t i=0; i<libusbTransfers.size(); i++)
			if (libusbTransfers[i]->endpoint==0x82)
				libusb_submit_transfer( libusbTransfers[i] );

		writeReg( 6,    0); // STREAM1_MODE

		writeReg(0x105, 0); // Disable auto-cycle of projector
		writeReg(26,    2); // PARAM_IR_RESOLUTION
		writeReg(27,   30); // PARAM_IR_FPS
		writeReg(18,    1); // PARAM_DEPTH_FORMAT
		writeReg(19,    1); // PARAM_DEPTH_RESOLUTION
		writeReg(20,   30); // PARAM_DEPTH_FPS
		writeReg(22, hole); // PARAM_DEPTH_HOLE_FILTER
		writeReg(36,  gmc); // PARAM_DEPTH_GMC_MODE
		writeReg(45, wBal); // PARAM_DEPTH_WHITE_BALANCE_ENABLE
		writeReg(62,    1); // PARAM_APC_ENABLE

		writeReg( 6,    2); // STREAM1_MODE
	}

	void stopDepth() {

		if( !isGood ) return;

		dInfo.enabled = false;

		writeReg( 6,    0); // STREAM1_MODE
	}

	void setDepthCB( void *cbd, void (*cb)( void *, uint8_t *, int) ) { dInfo.cb = cb; dInfo.cbd = cbd; }
	int  getDepthRAW( std::string &data, double timeOut=2. ) { return getRAW( dInfo, data, timeOut ); }

	static inline void dToDISP( std::string &in, std::vector<uint16_t> &out ) {

		out.clear();
		out.reserve(640*480);
		uint8_t*  i = (uint8_t*)&in[0];

//		struct T { static inline int pop( const uint8_t* i, uint32_t &c ) { return c&1?i[c++>>1]&15:i[c++>>1]>>4; } };
		struct T { static inline int pop( uint8_t *&i, uint32_t &c ) { return c++&1?*i++&15:*i>>4; } };

		int value = 0;
		uint32_t count = 0, inSize = in.size()*2;

		while (count<inSize) {

			int c = T::pop( i, count );
			if( c == 0xf ) {
				int aux  = (T::pop( i, count )<<4) + T::pop( i, count );
				if (aux & 0x80)
					value += aux-192;
				else
					value = (aux<<8) + (T::pop( i, count )<<4) + T::pop( i, count );
			} else if (c==0xe)
				for (int n=T::pop( i, count ); n; n--) out.push_back(value);
			else
				value += c-6;

			out.push_back(value);
		}
		while (out.size()<640*480) out.push_back(value);
	}

	int  getDepth( std::vector<uint16_t> &out, double timeOut=2.) { std::string d; if(!getDepthRAW(d, timeOut)) return 0; dToDISP(d,out); return 1; }



	static inline void cToYUV( std::string &in, std::vector<uint8_t> &out ) {

		out.resize(2000000);
		uint8_t* o = &out[0];
		uint8_t* i = (uint8_t*)&in[0];

		struct T { static inline int pop( const uint8_t* i, uint32_t &c ) { return c&1?i[c++>>1]&15:i[c++>>1]>>4; } };

		int value[4] = {0,0,0,0};
		uint32_t ch = 0, count = 0, inSize = in.size()*2;

		while( count<inSize ) {

			const int args[] = {1, 0, 2, 0};
			int &val = value[(args)[ch++%4]];

			int c = T::pop( i, count );
			if( c == 0xf )
				val  = (T::pop( i, count )<<4) + T::pop( i, count );
			else
				val += c-6;
			*o++ = val;
		}
		out.resize(o - &out[0]);
	}

	static inline void irTo16( const std::string &in, std::vector<uint16_t> &out ) {

		uint8_t *ip = (uint8_t *)&in[0];

		out.resize(in.size()*8/10);
		uint16_t *op = (uint16_t *)&out[0];

		for (uint32_t i=0; i<in.size(); i+=5) {
			*op++ = ((ip[i+0]<<2)|(ip[i+1]>>6))&1023;
			*op++ = ((ip[i+1]<<4)|(ip[i+2]>>4))&1023;
			*op++ = ((ip[i+2]<<6)|(ip[i+3]>>2))&1023;
			*op++ = ((ip[i+3]<<8)|(ip[i+4]>>0))&1023;
		}
	}

	void startImg( bool ir = false, int sz=1) {

		if (!isGood) return;

		if ( iInfo.enabled ) return; // Already enabled

		iInfo.enabled = true;
		//dInfo.enabled = false;

		for (uint32_t i=0; i<libusbTransfers.size(); i++)
			if (libusbTransfers[i]->endpoint==0x81)
				libusb_submit_transfer( libusbTransfers[i] );

		int f[]={60, 30, 15};
		if (ir) {
			if (sz!= 1) stopDepth();
			writeReg(0x105, 0); // Disable auto-cycle of projector
			writeReg(26,   sz); // PARAM_IR_RESOLUTION
			writeReg(27,f[sz]); // PARAM_IR_FPS
			writeReg( 5,    3);  // STREAM2_MODE
		} else {
			writeReg(26,    1); // PARAM_IR_RESOLUTION
			writeReg(27,   30); // PARAM_IR_FPS

			writeReg(12,    1);
			writeReg(13,    1);
			writeReg(14,   15);
			writeReg(71,    0); // disable Hflip
			writeReg( 5,    1);  // STREAM2_MODE
		}

		while (not iInfo.q.empty()) { Lock l(mtx); iInfo.q.pop(); }

		std::string s;
		getImgRAW( s );
	}

	void stopImg() {

		if (!isGood) return;

		iInfo.enabled = false;

		if (readReg(26)!=1) {
			writeReg(26,    1); // PARAM_IR_RESOLUTION
			writeReg(27,   30); // PARAM_IR_FPS
			writeReg( 5,    3);
			startDepth();
		}
		writeReg( 5,    0); // STREAM2_MODE
	}

	void setImgCB( void *cbd, void (*cb)( void *, uint8_t *, int) ) { iInfo.cb = cb; iInfo.cbd = cbd; }
	int  getImgRAW( std::string &data, double timeOut=2. ) { return getRAW( iInfo, data, timeOut ); }
	int  getImg( std::vector<uint16_t> &out, double timeOut=2.) { std::string d; if(!getImgRAW(d, timeOut)) return 0; irTo16(d,out); return 1; }
	int  getImg( std::vector<uint8_t > &out, double timeOut=2.) { std::string d; if(!getImgRAW(d, timeOut)) return 0; cToYUV(d,out); return 1; }


	void chechREGS() {

			fprintf(stderr,"\n");
			for( int j=0, i=0x010; i<0x120; i++) {
			int r = readReg(i);
			if( r ) fprintf(stderr, "reg=0x%03X:(%3d):val=0x%04X%c", i, i, r, (++j)%4?'\t':'\n');
		}
	}

	static std::vector<double> getDefaultDisp2Depth(double fx = 580) {

		std::function<double(double)> depth = [&](int i){ return  0.075*fx/(0.125*(1000.-i)); };

		std::vector<double> disp2depth(2048);
		for (int i=0; i<2048; i++)
			disp2depth[i]=depth(i);
		return disp2depth;
	}
};

