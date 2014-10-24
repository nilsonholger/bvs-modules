#ifndef DUO3D_H
#define DUO3D_H

#include <functional>

#include "bvs/module.h"
#include "DUOLib.h"
#include "opencv2/opencv.hpp"



/** This is the Duo3D module. It provides support for DUO3D devices.
 * Please add sufficient documentation to enable others to use it.
 *
 * Dependencies: opencv
 * Inputs: none
 * Outputs: outL and outR (cv::Mat with left and right image)
 * Configuration Options: please see Duo3D.conf
 *
 * TODO: output all available duo data: timeStamp, accel, gyro, mag and temp
 * TODO: SetDUOLedPWMSeq(duo, val, size)
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

		DUOInstance duo = NULL; /**< DUO device instance. */
		DUOResolutionInfo duo_res; /**< DUO resolution info object. */
		static PDUOFrame duo_frame; /**< Frame with current image data. */
		static void DUOCallback(const PDUOFrame pFrameData, void* pUserData); /**< Callback to update PDUOFrame. */

		bool showDuoInfo; /**< Show DUO device info at startup. */
		bool showDuoParams; /**< Show DUO parameters used by device. */

		int width; /**< Selected image width. */
		int height; /**< Selected image height. */
		int binning; /**< Selected binning mode. */
		float fps; /**< Selected frames per second. */

		/** Checks DUO function success and prints error otherwise.
		 * @param[in] success Whether the DUO function was successfull.
		 * @param[in] parameterName Name of parameter for error message.
		 */
		void setParam(bool success, std::string parameterName);

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
		LOG(1, message << t);
	} else {
		LOG(0, "Could not set: " << message.erase(message.find_first_of(':'), std::string::npos));
	}
}



#endif //DUO3D_H

