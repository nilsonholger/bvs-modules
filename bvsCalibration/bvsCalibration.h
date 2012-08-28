#ifndef BVSCALIBRATION_H
#define BVSCALIBRATION_H

#include<chrono>
#include<condition_variable>
#include<mutex>
#include<thread>
#include<vector>

#include "bvs/module.h"
#include "calibrationnode.h"
#include "stereocalibration.h"
#include "calibrationguide.h"
#include "opencv2/opencv.hpp"



/** This is the bvsCalibration class.
 * This method will calibrate a variable number of cameras, called nodes
 * within this class. So far, only stereo (2 nodes) is supported.
 * For each node, 2 BVS::Connector objects will be created automatically,
 * input0 and output0, with 0 being replaced by the node id (0, 1, 2, ...).
 * The number of nodes can be set in the supplied config file.
 *
 * To compile this module, opencv(2) headers and libs are needed.
 *
 * Please see the config file for more options.
 *
 * TODO load settings from OWN config file and add comments to it
 * TODO display validRegionOfInterest or crop images...
 */
class bvsCalibration : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] id Your modules unique identifier, will be set by framework.
		 * @param[in] bvs Reference to info for e.g. option retrieval.
		 */
		bvsCalibration(const std::string id, const BVS::Info& bvs);

		/** Your module destructor. */
		~bvsCalibration();

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
		 * @param[in] addGridOverlay Whether to draw a grid on the output images.
		 */
		void rectifyOutput(bool addGridOverlay = false);



		const std::string id; /**< Your unique module id, set by framework. */
		BVS::Logger logger; /**< Logger instance. */
		const BVS::Info& bvs; /**< BVS information reference. */

		// config options
		int numNodes; /**< Number of calibration nodes. */
		int numImages; /**< Number of calibration images. */
		float circleSize; /**< Circle size of detection pattern. */
		bool autoShotMode; /**< Auto or manual mode. */
		int autoShotDelay; /**< Delay for auto mode. */
		std::string directory; /**< Base directory of calibration data. */
		bool saveImages; /**< Whether to save images or not. */
		bool useSavedImages; /**< Calculate calibration using saved images. */
		bool loadCalibration; /**< Whether to load calibration. */
		bool saveCalibration; /**< Whether to save calibration. */
		std::string calibrationFile; /**< File to save/load calibration to/from. */
		bool createRectifiedOutput; /**< Whether to rectify the output images. */
		bool addGridOverlay; /**< Whether to add grid overlay on output images. */
		bool useCalibrationGuide; /**< Whether to use the calibration guide. */
		float centerScale; /**< Scale of center detections in percent of image size. */
		int centerDetections; /**< Number of center detections. */
		int sectorDetections; /**< Number of per sector detections. */

		bool calibrated; /** Calibration status. */
		bool detectionRunning; /** Detection thread running in background. */
		int numDetections; /**< Number of detected patterns. */

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

		bvsCalibration(const bvsCalibration&) = delete; /**< -Weffc++ */
		bvsCalibration& operator=(const bvsCalibration&) = delete; /**< -Weffc++ */
};



#endif //BVSCALIBRATION_H

