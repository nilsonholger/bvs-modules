#ifndef DUO3D_H
#define DUO3D_H

#include <array>
#include <chrono>
#include <functional>
#include <thread>

#include "bvs/module.h"
#include "DUOLib.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"



/** This is the Duo3D module. It provides support for DUO3D devices.
 * Dependencies: opencv
 * Inputs: none
 * Outputs:
 *  - outL and outR (cv::Mat with left and right image)
 *  - outTime (unsigned __int32)
 *  - outAccel (float[3])
 *  - outGyro (float[3])
 *  - outMag (float[3])
 *  - outTemp (float)
 *  - outDUOFrame (struct DUOFrame from DUO API, includes all of the above)
 * Configuration Options: please see Duo3D.conf
 */
class Duo3D : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		Duo3D(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~Duo3D();

		/** Execute function doing all the work.
		 * This function is executed exactly once during each started
		 * round/step of the framework. It is supposed to contain the actual
		 * work of your module.
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

		BVS::Connector<cv::Mat> outL; /**< Left image output. */
		BVS::Connector<cv::Mat> outR; /**< Right image output. */
		BVS::Connector<uint32_t> outTime; /**< Time stamp information. */
		BVS::Connector<std::array<float, 3>> outAccel; /** Accelerometer data. */
		BVS::Connector<std::array<float, 3>> outGyro; /** Gyroscope data. */
		BVS::Connector<std::array<float, 3>> outMag; /** Magnetometer data. */
		BVS::Connector<float> outTemp; /** Temperature data. */
		BVS::Connector<DUOFrame> outDUOFrame; /** All of the above in a single struct. */

		DUOInstance duo = NULL; /**< DUO device instance. */
		DUOResolutionInfo duo_res; /**< DUO resolution info object. */
		static PDUOFrame duo_frame; /**< Frame with current image data. */
		static void DUOCallback(const PDUOFrame pFrameData, void* pUserData); /**< Callback to update PDUOFrame. */
		static std::mutex mutex; /**< Mutex to synchronize with DUOCallback. */

		bool showDuoInfo; /**< Show DUO device info at startup. */
		bool showDuoParams; /**< Show DUO parameters used by device. */
		bool showDuoFOV; /**< Show DUO's field of view parameters. */
		bool showDuoStereo; /**< Show DUO's builtin stereo calibration details. */
		bool blockModule; /**< Whether to block the module n ms after sending a frame. */

		int width; /**< Selected image width. */
		int height; /**< Selected image height. */
		int binning; /**< Selected binning mode. */
		float fps; /**< Selected frames per second. */
		bool undistort; /** Enable builtin image rectification. */
		bool autoCorrect; /**< Automagically update gain/exposure. */
		double autoQuantile; /**< Remove quantiles when calculating mean. */
		unsigned int autoTargetMean; /** Target mean for auto correction. */
		double autoAttenuation; /**< Attenuation factor for gain correction. */
		bool antiReadNoise; /**< Reduce CCD read noise. */
		std::string noiseFrameFileLeft; /**< Path to left noise frame. */
		std::string noiseFrameFileRight; /**< Path to right noise frame. */
		cv::Mat noiseLeft; /**< Noise for left camera. */
		cv::Mat noiseRight; /**< Noise for right camera. */

		unsigned int timeStamp; /**< Duo frame time stamp. */

		/** Checks DUO function success and prints error otherwise.
		 * @param[in] success Whether the DUO function was successfull.
		 * @param[in] parameterName Name of parameter for error message.
		 */
		void setParam(bool success, std::string parameterName);

		/** Perform automagic(TM) image correction.
		 */
		void autoCorrection();

		/** Reduce CCD read noise.
		 */
		void readNoiseCorrection();

		/** Checks DUO function success and prints retrieved parameter.
		 * @param[in] success Whether the DUO function was successfull.
		 * @param[in] t Retrieved parameter value.
		 * @param[in] message Message to display when showing parameter.
		 */
		template<typename T> void printParam(bool success, T& t, std::string message);

		Duo3D(const Duo3D&) = delete; /**< -Weffc++ */
		Duo3D& operator=(const Duo3D&) = delete; /**< -Weffc++ */
};



template<typename T> void Duo3D::printParam(bool success, T& t, std::string message)
{
	if (success) {
		LOG(2, message << t);
	} else {
		LOG(0, "Could not set: " << message.erase(message.find_first_of(':'), std::string::npos));
	}
}



#endif //DUO3D_H

