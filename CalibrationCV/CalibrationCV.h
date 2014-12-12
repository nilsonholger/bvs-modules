#ifndef CALIBRATIONCV_H
#define CALIBRATIONCV_H

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "bvs/module.h"
#include "calibrationnode.h"
#include "stereocalibration.h"
#include "calibrationguide.h"
#include "opencv2/opencv.hpp"



/** This is the CalibrationCV module.
 * This module will calibrate a variable number of cameras, called nodes
 * within this module.
 *
 * IMPORTANT: So far, only stereo (2 nodes) is supported.
 *
 * Dependencies: opencv
 * Inputs: in<N>, where <N> is a node id starting with 1
 * Outputs: out<N>, where <N> is a node id starting with 1
 * Configuration Options: please see CalibrationCV.conf
 */
class CalibrationCV : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to info for e.g. option retrieval.
		 */
		CalibrationCV(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~CalibrationCV();

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
		/** Load calibration from directory/file.
		 * @param[in] directory Directory of target file.
		 * @param[in] file Target file name.
		 * @return True if successfull, false otherwise.
		 */
		bool loadCalibrationFrom(const std::string& directory, const std::string& file);

		/** Save calibration to directory/file.
		 * @param[in] directory Directory of target file.
		 * @param[in] file Target file name.
		 * @return True if successfull, false otherwise.
		 */
		bool saveCalibrationTo(const std::string& directory, const std::string& file);

		/** Generate reflection map.
		 * This function generates the reflection map used to reflect output images
		 * while collecting calibration images.
		 */
		void generateReflectionMap();

		/** Calibrate node(s).
		 * Depending on the number of node(s) (cameras), the corresponding
		 * handler is called to calibrate them.
		 */
		void calibrate();

		/** Collect calibration Images.
		 * This methods collects calibration images. To increase the framerate
		 * it works on downscaled versions, detects calibration patterns and
		 * rescales the detections to show them in the original size.
		 * If a positive detection is found, notifyDetectionThread() is called.
		 */
		void collectCalibrationImages();

		/** Notifies the detection thread.
		 * This method prepares the input and notifies the detection thread.
		 */
		void notifyDetectionThread();

		/** Detect calibration points.
		 * This function is called by the detection thread. The input is checked
		 * for valid detections for all nodes. If that is positive, the detected
		 * points are stored and numDetections is increased.
		 */
		void detectCalibrationPoints();

		/** Clear calibration data.
		 * Resets all collected data after calibration is successfull.
		 */
		void clearCalibrationData();

		/** Rectify output.
		 * This method rectifies the output for all nodes.
		 */
		void rectifyOutput();

		/** Rectify calibration images. */
		bool rectifyCalibrationImages();



		const BVS::ModuleInfo info; /**< Your module metadata, set by framework. */
		BVS::Logger logger; /**< Logger instance. */
		const BVS::Info& bvs; /**< BVS information reference. */

		// config options
		int numNodes; /**< Number of calibration nodes. */
		int numImages; /**< Number of calibration images. */
		float blobSize; /**< Square size or circle center distance of detection pattern. */
		bool autoShotMode; /**< Auto or manual mode. */
		int autoShotDelay; /**< Delay for auto mode. */
		std::string directory; /**< Base directory of calibration data. */
		bool saveImages; /**< Whether to save images or not. */
		bool useSavedImages; /**< Calculate calibration using saved images. */
		bool rectifyCalImages; /**< Save rectified calibration images and quit. */
		std::string imageDirectory; /**< Directory for calibration images (saveImages/useSavedImages). */
		std::string outputDirectory; /** Directory for rectified calibration images. */
		bool loadCalibration; /**< Whether to load calibration. */
		bool saveCalibration; /**< Whether to save calibration. */
		std::string calibrationFile; /**< File to save/load calibration to/from. */
		bool createRectifiedOutput; /**< Whether to rectify the output images. */
		bool addGridOverlay; /**< Whether to add grid overlay on output images. */
		bool useCalibrationGuide; /**< Whether to use the calibration guide. */
		int sectorDetections; /**< Number of per sector detections. */
		bool fisheye; /**< Use fisheye model for calibration and rectification. */

		bool calibrated; /** Calibration status. */
		bool detectionRunning; /** Detection thread running in background. */
		int numDetections; /**< Number of detected patterns. */
		int rectifyCounter; /**< Number of rectified images. */

		std::vector<std::vector<cv::Point3f>> objectPoints; /**< Vector of pattern object points. */
		cv::Size imageSize; /**< Input image size. */
		cv::Size boardSize; /**< Detection pattern board size. */

		std::thread detectionThread; /**< Detection thread handle. */
		std::mutex detectionMutex; /**< Mutex for detectionThread. */
		std::unique_lock<std::mutex> detectionLock; /**< Lock for detectionThread. */
		std::condition_variable detectionCond; /**< Condition variable for detectionThread. */

		/** Shot timer (for auto shot mode time delay). */
		std::chrono::time_point<std::chrono::high_resolution_clock> shotTimer;

		cv::Mat reflectX; /** Reflection map. */
		cv::Mat reflectY; /** Reflection map. */

		CalNodeVec nodes; /**< Vector of calibraton nodes metadata. */
		StereoCalibration stereo; /**< Stereo calibration */
		CalibrationGuide guide; /** Calibration guidance system. */

		CalibrationCV(const CalibrationCV&) = delete; /**< -Weffc++ */
		CalibrationCV& operator=(const CalibrationCV&) = delete; /**< -Weffc++ */
};



#endif //CALIBRATIONCV_H

