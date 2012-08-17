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
#include "opencv2/opencv.hpp"



/** This is the bvsCalibration class.
 * Please add sufficient documentation to enable others to use it.
 * Include information about:
 * - Dependencies
 * - Inputs
 * - Outputs
 * - Configuration Options
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
		const std::string id; /**< Your unique module id, set by framework. */

		/** Your logger instance.
		 * @see Logger
		 */
		BVS::Logger logger;

		/** Your Info reference.
		 * @see Info
		 */
		const BVS::Info& bvs;

		//TODO comment
		bool calibrated;
		bool detectionRunning;

		//TODO comment settings
		int numNodes;
		int numImages;
		int numDetections;
		float circleSize;
		bool autoShotMode;
		int autoShotDelay;
		std::string directory;
		bool saveImages;
		bool loadCalibration;
		bool saveCalibration;
		std::string calibrationFile;
		bool createRectifiedOutput;
		bool addGridOverlay;

		CalNodeVec nodes;

		std::vector<std::vector<cv::Point3f>> objectPoints;
		cv::Size imageSize;
		cv::Size boardSize;

		StereoCalibration stereo;

		std::thread detectionThread;
		std::mutex detectionMutex; /**< Mutex for detectionThread. */
		std::unique_lock<std::mutex> detectionLock; /**< Lock for detectionThread. */
		std::condition_variable detectionCond; /**< Condition variable for detectionThread. */

		std::chrono::time_point<std::chrono::steady_clock> shotTimer;

		bool loadCalibrationFrom(const std::string& directory, const std::string& file);
		bool saveCalibrationTo(const std::string& directory, const std::string& file);
		void calibrate();

		void collectCalibrationImages();
		void notifyDetectionThread();
		void detectCalibrationPoints();
		void clearCalibrationData();

		void rectifyOutput(bool addGridOverlay = false);

		bvsCalibration(const bvsCalibration&) = delete; /**< -Weffc++ */
		bvsCalibration& operator=(const bvsCalibration&) = delete; /**< -Weffc++ */
};

#endif //BVSCALIBRATION_H

