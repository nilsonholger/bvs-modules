#ifndef KINECTXLITE_H
#define KINECTXLITE_H

#include <vector>

#include "bvs/module.h"
#include "opencv2/opencv.hpp"

// Many thanks to Manel.
#include "kinectLite.hpp"



/** This is the KinectXLite module.
 * It delivers disparity and depth information from a Kinect sensor.
 *
 * Dependencies: Kinect device, OpenCV
 * Inputs: -
 * Outputs: outDepth, outDisp
 * Configuration Options:
 */
class KinectXLite : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		KinectXLite(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~KinectXLite();

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

		KinectLite kinect; /**< KinectLite interface. */
		std::vector<double> disp2Depth; /**< Disparity to depth conversion lookup vector. */
		std::vector<uint16_t> dispVec; /**< Disparity vector returned by KinectLite. */
		cv::Mat depth; /**< Depth map. */
		cv::Mat disp; /**< Disparity map. */
		double baseline; /** Baseline of virtual stereo camera. */
		int focalLength; /** Focal length (Fx) of virtual camera. */

		bool requestShutdown; /**< True if shutdown necessary. */
		BVS::Connector<cv::Mat> outDepth; /**< Output for depth information. */
		BVS::Connector<cv::Mat> outDisp; /**< Output for disparity information. */

		KinectXLite(const KinectXLite&) = delete; /**< -Weffc++ */
		KinectXLite& operator=(const KinectXLite&) = delete; /**< -Weffc++ */
};



#endif //KINECTXLITE_H

