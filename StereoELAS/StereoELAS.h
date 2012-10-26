#ifndef STEREOELAS_H
#define STEREOELAS_H

#include "bvs/module.h"
#include "opencv2/opencv.hpp"
#include "elas.h"
#include <atomic>
#include <condition_variable>
#include <thread>
#include <vector>



/** This is the StereoELAS class.
 * Please add sufficient documentation to enable others to use it.
 * Include information about:
 * - Dependencies
 * - Inputs
 * - Outputs
 * - Configuration Options
 */
class StereoELAS : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] id Your modules unique identifier, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		StereoELAS(const std::string id, const BVS::Info& bvs);

		/** Your module destructor. */
		~StereoELAS();

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

		/** Your Info reference;
		 * @see Info
		 */
		const BVS::Info& bvs;

		/** Example Connector used to retrieve/send data from/to other modules.
		 * @see Connector
		 */
		BVS::Connector<cv::Mat> inL;
		BVS::Connector<cv::Mat> inR;
		BVS::Connector<cv::Mat> outL;
		BVS::Connector<cv::Mat> outR;

		int discardTopLines;
		int discardBottomLines;
		float scalingFactor;
		int sliceCount;
		int sliceOverlap;
		bool sliceExit;

		std::atomic<int> runningThreads;
		std::mutex masterMutex;
		std::mutex sliceMutex;
		std::unique_lock<std::mutex> masterLock;
		std::condition_variable monitor;
		std::condition_variable threadMonitor;
		std::vector<std::thread> threads;
		std::vector<bool> flags;

		cv::Mat tmpL;
		cv::Mat tmpR;
		cv::Mat left;
		cv::Mat right;
		cv::Mat dispL;
		cv::Mat dispR;
		int dimensions[3];
		Elas::parameters param;
		Elas elas;

		void sliceThread(int id);

		StereoELAS(const StereoELAS&) = delete; /**< -Weffc++ */
		StereoELAS& operator=(const StereoELAS&) = delete; /**< -Weffc++ */
};

#endif //STEREOELAS_H

