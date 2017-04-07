#include "GPSParser.h"

#include <cstdio>
#include <functional>



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
GPSParser::GPSParser(BVS::ModuleInfo info, const BVS::Info& _bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(_bvs)
	, verbose{bvs.config.getValue<bool>(info.conf + ".verbose", false)}
	, interface{bvs.config.getValue<std::string>(info.conf + ".interface", {})}
	, console{}
	, checksum_match{true}
	, consoleListenerThread{}
	, mutex{}
	, shutdown{false}
	, out("gps-data", BVS::ConnectorType::OUTPUT)
{
	consoleListenerThread = std::thread{&GPSParser::consoleListener, this};
}



// This is your module's destructor.
GPSParser::~GPSParser() noexcept
{
	shutdown = true;
	if (consoleListenerThread.joinable()) consoleListenerThread.join();
}



// Put all your work here.
BVS::Status GPSParser::execute()
{
	if (out.active())
	{
		std::lock_guard<std::mutex> lock{mutex};
		out_data = {
			{"stat", temp[0]},
			{"date", temp[1]},
			{"time", temp[2]},
			{"lat", temp[3]},
			{"lon", temp[4]},
			{"skn", temp[5]},
			{"skph", temp[6]},
			{"cot", temp[7]},
			{"com", temp[8]},
			{"sats", temp[9]},
			{"pdop", temp[10]},
			{"hdop", temp[11]},
			{"vdop", temp[12]},
			{"amsl", temp[13]},
			{"ageo", temp[14]}
		};
	}

	return BVS::Status::OK;
}



GPSParser& GPSParser::consoleListener()
{
	BVS::nameThisThread("GPSlistener");

	console.open(interface, std::ios_base::in | std::ios_base::ate);
	console.rdbuf()->pubsetbuf(0, 0);
	if (!console.is_open()) {
		LOG(0, "could not open serial console interface from " << interface);
	}

	// checksum calculation functional
	std::function<bool(std::string&)> calc_checksum_state = [&](std::string str) {
		int checksum = 0;
		const char *s = str.substr(1, str.length()-5).c_str();
		while(*s) checksum^= *s++;
		if (checksum != std::stoi(str.substr(str.length()-3, 2), nullptr, 16)) {
			LOG(1, "NMEA sentence checksum failure!");
			return false;
		} else {
			return true;
		}
	};

	// local storage
	char valid = ' ';                    // data status
	double fix_d = 0, fix_t = 0;         // fix date and time
	double lat = 0, lon = 0;             // latitude and longitude in (d)ddmm.mm
	char lat_h = ' ', lon_h = ' ';       // lat/lon hemisphere
	double speed_kn = 0, speed_kph = 0;  // ground speed in knots and kilometers per hour
	double course_t = 0, course_m = 0;   // course true/magnetic
	int satellites = 0;                  // number of satellites used
	double pdop = 0, hdop = 0, vdop = 0; // position/horizontal/vertical dilution of precision
	double alt_amsl = 0, alt_geo = 0;    // altitude above mean sea level and geoidal

	// parse messages
	std::string sentence;
	while (!shutdown) {
		if (!console.fail()) std::getline(console, sentence);
		else LOG(0, "lost connection to " << interface);

		// parse content
		std::string type = sentence.substr(1,5);
		/* $GPRMC: recommended miminum specific GPS data
		 * $GPVTG: track made good and ground speed
		 * $GPGGA: global positioning system fix data
		 * $GPGSA: dilution of precision and active satellites
		 * $GPGLL: geographic position, latitude/longitude and time
		 *
		 * sentence formats:
		 * $GPRMC,hhmmss.ss,d,ddmm.mm,a,dddmm.mm,b,n.n,t.t,ddmmyy,v.v,a,M*CS
		 * $GPVTG,t.t,T,m.m,M,n.n,N,k.k,K,M*CS
		 * $GPGGA,hhmmss.ss,ddmm.mmm,a,dddmm.mmm,b,q,ss,h.h,a.a,M,g.g,M,d.d,nnnn*CS
		 * $GPGSA,m,f,s,s,s,s,s,s,s,s,s,s,s,s,p.p,h.h,v.v*CS
		 * $GPGLL,ddmm.mm,a,dddmm.mm,b,hhmmss.ss,d,M*CS
		 *
		 * data:
		 * d          - Data status (A=Valid position, V=navigation receiver warning)
		 * hhmmss.ss  - UTC time of fix
		 * ddmmyy     - UTC date of fix
		 * ddmm.mm,a  - Latitude of fix, N or S
		 * dddmm.mm,b - Longitude of fix, E or W
		 * t.t(,T)    - True course made good over ground, degrees
		 * m.m(,M)    - Magnetic course made good over ground, degrees
		 * n.n(,N)    - Ground speed, N=Knots
		 * k.k(,K)    - Ground speed, K=Kilometers per hour
		 * v.v,a      - Magnetic variation degrees (Easterly var. subtracts from true course), E or W of magnetic variation
		 * M          - Mode indicator, (A=Autonomous, D=Differential, E=Estimated, N=Data not valid)
		 * m          - Mode: M=Manual, forced to operate in 2D or 3D, A=Automatic 3D/2D
		 * f          - fix Mode: 1=Fix not available, 2=2D, 3=3D
		 * q          - GPS Quality indicator (0=No fix, 1=Non-differential GPS fix, 2=Differential GPS fix, 6=Estimated fix)
		 * ss         - number of satellites in use
		 * s          - Satellite vechicles (SV's) PRN's used in position fix (null for unused fields)
		 * p.p        - Position Dilution of Precision (PDOP)
		 * h.h        - Horizontal Dilution of Precision (HDOP)
		 * v.v        - Vertical Dilution of Precision (VDOP)
		 * a.a,M      - Antenna altitude above mean-sea-level (Meters)
		 * g.g,M      - Geoidal height (Meters)
		 * d.d        - Age of Differential GPS data (seconds since last valid RTCM transmission)
		 * nnnn       - Differential reference station ID, 0000 to 1023
		 * CS         - Checksum
		 */
		if (type=="GPRMC") {
			checksum_match &= calc_checksum_state(sentence);
			if (checksum_match) {
				sscanf(sentence.c_str(), "$GPRMC,%lf,%c,%lf,%c,%lf,%c,%lf,%lf,%lf,%*s", &fix_t, &valid, &lat, &lat_h, &lon, &lon_h, &speed_kn, &course_t, &fix_d);
				//LOG(1, fix_t << " " << valid << " " << lat << lat_h << " " << lon << lon_h << " " << speed_kn << " " << course_t << " " << fix_d);
			}
		} else if (type=="GPVTG") {
			checksum_match &= calc_checksum_state(sentence);
			if (checksum_match) {
				sscanf(sentence.c_str(), "$GPVTG,%lf,T,%lf,M,%lf,N,%lf,K,%*s", &course_t, &course_m, &speed_kn, &speed_kph);
				//LOG(1, course_t << " " << course_m << " " << speed_kn << " " << speed_kph << " " << mode);
			}
		} else if (type=="GPGGA") {
			checksum_match &= calc_checksum_state(sentence);
			if (checksum_match) {
				sscanf(sentence.c_str(), "$GPGGA,%lf,%lf,%c,%lf,%c,%*c,%d,%lf,%lf,M,%lf,M,%*s", &fix_t, &lat, &lat_h, &lon, &lon_h, &satellites, &hdop, &alt_amsl, &alt_geo);
				//LOG(1, fix_t << " " << lat << lat_h << " " << lon << lon_h << " " << quality << " " << satellites << " " << hdop << " " << alt_amsl << "m " << alt_geo << "m");
			}
		} else if (type=="GPGSA") {
			checksum_match &= calc_checksum_state(sentence);
			if (checksum_match) {
				std::string dop = sentence.substr(sentence.rfind(',', sentence.find_first_of('.')), std::string::npos);
				sscanf(dop.c_str(), ",%lf,%lf,%lf,%*s", &pdop, &hdop, &vdop);
				//LOG(1, pdop << " " << hdop << " " << vdop);
			}
		} else if (type=="GPGLL") { // updates provided GPS data
			checksum_match &= calc_checksum_state(sentence);
			if (checksum_match) {
				sscanf(sentence.c_str(), "$GPGLL,%lf,%c,%lf,%c,%lf,%c,%*s", &lat, &lat_h, &lon, &lon_h, &fix_t, &valid);
				//LOG(1, lat << lat_h << " " << lon << lon_h << " " << fix_t << " " << valid << " " << mode);

				// update data
				if (valid == 'V') LOG(1, "navigation receiver warning (no/invalid data)!");
				std::lock_guard<std::mutex> lock{mutex};
				temp = {{
					double(valid&0b1), // valid: 'A'==65, 'V'==86, valid&0b1==mod2
					fix_d, fix_t,
					lat * (1-2*(lat_h&0b1)), // lat_h: 'N'==78, 'S'==83, lat_h&0b1==mod2
					lon * (1-(lon_h&0b10)), // lon_h: 'E'==69, 'W'==87
					speed_kn, speed_kph,
					course_t, course_m,
					double(satellites),
					pdop, hdop, vdop,
					alt_amsl, alt_geo
				}};
			} else {
				checksum_match = true;
			}
		}
		if (verbose) LOG(3, sentence);
	}
	console.close();

	return *this;
}



// UNUSED
BVS::Status GPSParser::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(GPSParser)

