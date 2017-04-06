#ifndef GPSPARSER_H
#define GPSPARSER_H

#include <array>
#include <fstream>
#include <map>
#include <mutex>
#include <thread>

#include "bvs/module.h"



/** This is the GPSParser module.
 * This module parses GPS information from USB attached GNSS/GPS receivers that
 * provide their data in text format over a serial console. Supported format is
 * NMEA 0183 version 3.00 [1] (with active Mode indicator field). It collects
 * data from $GPRMC, $GPVTG, $GPGGA and $GPGLL sentences. All four must be
 * present or data will be incomplete. Only $GPGLL occurences will update the
 * provided GPS data. See [2] for NMEA data sentence examples and explanations.
 *
 * [1] http://www.nmea.org
 * [2] http://home.mira.net/~gnb/gps/nmea.html
 *
 * Dependencies: none.
 * Outputs: out
 * Configuration Options: please see GPSParser.conf
 *
 * Output format:
 * Out provides a std::map<std::string, double>, which can be used to retrieve
 * various GPS provided data:
 *    "date" -> date of last GPS fix (valid received position)
 *    "time" -> time of last fix
 *    "lat"  -> latitude in (-)ddmm.mm (degrees and minutes, '-' denotes 'S')
 *    "lon"  -> longitude in (-)dddmm.mm ('-' denotes 'W')
 *    "skn" -> ground speed in knots
 *    "skph" -> ground speed in kilometers per hour
 *    "cot" -> true course
 *    "com" -> magnetic course
 *    "sats" -> number of satellites used
 *    "hdop" -> horizontal dilution of precision
 *    "amsl" -> altitude above mean sea level
 *    "ageo" -> geoidal altitude
		};
 */
class GPSParser : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		GPSParser(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~GPSParser() noexcept;

		/** Execute function doing all the work.
		 * This function is executed exactly once during each started
		 * round/step of the framework. It is supposed to contain the actual
		 * work of your module.
		 * @return Module's status.
		 */
		BVS::Status execute();

		/** UNUSED
		 * @return Module's status.
		 */
		BVS::Status debugDisplay();

	private:
		const BVS::ModuleInfo info; /**< Your module metadata, set by framework. */
		BVS::Logger logger; /**< Your logger instance. @see Logger */
		const BVS::Info& bvs; /**< Your Info reference. @see Info */

		bool verbose; /**< Whether to log each received NMEA sentence (level 3). */
		std::string interface; /**< The serial console path. */
		std::ifstream console; /**< The serial console (ifstream). */
		bool checksum_match; /**< Whether there was a checksum (mis-)match. */

		std::thread consoleListenerThread; /**< Thread for console listener. */
		std::mutex mutex; /**< Mutex to synchronize data access. */
		bool shutdown; /**< Signals console thread to close device. */

		GPSParser& consoleListener();

		std::array<double, 12> temp;
		std::map<std::string, double> out_data;

		/** Example Connector used to retrieve/send data from/to other modules.
		 * @see Connector
		 */
		BVS::Connector<std::map<std::string, double>> out;

		GPSParser(const GPSParser&) = delete; /**< -Weffc++ */
		GPSParser& operator=(const GPSParser&) = delete; /**< -Weffc++ */
};



#endif //GPSPARSER_H

