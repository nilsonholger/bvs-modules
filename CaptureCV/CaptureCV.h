#ifndef CAPTURECV_H
#define CAPTURECV_H

#include "bvs/module.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>



/** This is the CaptureCV module.  It can capture input from a given amount of
 * cameras, videos or image files, as well as write video from its inputs to
 * individual video files.
 *
 * Dependencies: opencv
 * Inputs: in<N>, where <N> is a node id starting with 1
 * Outputs: out<N>, where <N> is a node id starting with 1
 * Configuration Options: please see CaptureCVConfig.txt
 */
class CaptureCV : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] id Your modules unique identifier, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		CaptureCV(const std::string id, const BVS::Info& bvs);

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
		const std::string id; /**< Your unique module id, set by framework. */
		BVS::Logger logger; /**< Logger. */
		const BVS::Info& bvs; /**< Framework info. */
		std::vector<BVS::Connector<cv::Mat>*> outputs; /**< Output connectors. */
		std::vector<BVS::Connector<cv::Mat>*> inputs; /**< Input connectors. */
		std::vector<cv::VideoCapture> captures; /**< Captures vector for camera usage. */
		std::vector<cv::VideoWriter> writers; /**< Writers vector for video output. */
		int numNodes; /**< Number of nodes to use. */
		bool useVideo; /**< Use videos as input sources. */
		std::vector<std::string> videoList; /**< List of video names to use. */
		bool useImages; /**< Use images as input sources. */
		std::string imageNameScheme; /**< Image naming scheme. */
		std::vector<std::string> nameParts; /**< Individual name scheme parts. */
		int imageCounter; /**< Image counter, used when reading from images. */
		int captureMode; /**< Capture mode, not always supported by capture devices. */
		double captureFPS; /**< Capture frames per second, not always supported. */
		bool recordVideo; /**< Record videos from inputs. */
		std::string recordFOURCC; /**< FOURCC to set in recorded videos. */
		double recordFPS; /**< FPS to set in recorded videos. */
		int recordWidth; /**< Recorded video width. */
		int recordHeight; /**< Recorded video height. */
		bool recordColor; /**< Record videos with color, or not. */
		int fourcc; /**< FOURCC value as integer representation. */

		/** Get filename using image name scheme.
		 * @param[in] frame Frame number to use.
		 * @param[in] nodeID Node id to use.
		 */
		std::string getFileNameFromParts(int frame, int nodeID);

		CaptureCV(const CaptureCV&) = delete; /**< -Weffc++ */
		CaptureCV& operator=(const CaptureCV&) = delete; /**< -Weffc++ */
};



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(CaptureCV)



#endif //CAPTURECV_H

