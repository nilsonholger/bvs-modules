#ifndef CAPTURECV_H
#define CAPTURECV_H

#include "bvs/module.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <string>
#include <vector>



/** This is the CaptureCV module.  It can capture input from a given amount of
 * cameras, videos or image files, as well as write video from its inputs to
 * individual video files.
 *
 * Dependencies: opencv
 * Inputs: in<N>, where <N> is a node id starting with 1
 * Outputs: out<N>, where <N> is a node id starting with 1
 * Configuration Options: please see CaptureCV.conf
 */
class CaptureCV : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		CaptureCV(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~CaptureCV();

		/** Execute function doing all the work.
		 * This function is executed exactly once and only once upon each started
		 * round/step of the framework. It is supposed to contain the actual work
		 * of your module.
		 */
		BVS::Status execute();

		/** UNUSED
		 * @return Module's status.
		 */
		BVS::Status debugDisplay();

	private:
		const BVS::ModuleInfo info; /**< Your module metadata, set by framework. */
		BVS::Logger logger; /**< Logger. */
		const BVS::Info& bvs; /**< Framework info. */
		std::vector<BVS::Connector<cv::Mat>*> outputs; /**< Output connectors. */
		std::vector<BVS::Connector<cv::Mat>*> inputs; /**< Input connectors. */
		BVS::Connector<std::string> folder; /** Optional folder name input. */
		std::vector<cv::VideoCapture> captures; /**< Captures vector for camera usage. */
		std::vector<cv::VideoWriter> writers; /**< Writers vector for video output. */
		int numNodes; /**< Number of nodes to use. */
		int nodeOffset; /**< When capturing from camera, start with this id. */
		char mode; /**< Capture mode to use (camera, video, image, none). */
		bool displayMode; /**< Enable disable immediate data display. */
		std::vector<std::string> videoFiles; /**< List of video names to use. */
		std::string imageFiles; /**< Image files naming scheme. */
		int frameNumberPadding; /**< Width of frame Number (Zero Padding) when used as string. */
		std::vector<std::string> fileNamePieces; /**< Individual image name scheme parts. */
		int counterStart; /**< Frame counter start value, used when reading from images. */
		int stepSize; /**< Frame counter step size. */
		int cameraMode; /**< Capture mode, not always supported by capture devices. */
		double cameraFPS; /**< Capture frames per second, not always supported. */
		int cameraWidth; /**< Capture width, not always supported. */
		int cameraHeight; /**< Capture height, not always supported. */
		std::string recordFOURCC; /**< FOURCC to set in recorded videos. */
		int fourcc; /**< FOURCC value as integer representation. */
		double recordFPS; /**< FPS to set in recorded videos. */
		bool recordColor; /**< Record videos with color, or not. */

		/** Get filename using image name scheme.
		 * @param[in] frame Frame number to use.
		 * @param[in] nodeID Node id to use.
		 */
		std::string getImageFileName(int frame, int nodeID);

		CaptureCV(const CaptureCV&) = delete; /**< -Weffc++ */
		CaptureCV& operator=(const CaptureCV&) = delete; /**< -Weffc++ */
};



#endif //CAPTURECV_H

