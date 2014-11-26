#ifndef FLOWERBOXREADER_H
#define FLOWERBOXREADER_H

#include<string>
#include<vector>

#include "bvs/module.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"



/** This is the FlowerBoxReader module.
 * It provides streamlined access to the FlowerBox dataset from
 * https://cvhci.anthropomatik.kit.edu/~dkoester/data/flowerbox/
 * It will notify of any missing/not found files and only try to load those
 * parts of the dataset, that are actually accessed through the module's
 * outputs.
 *
 * Dependencies: opencv
 * Outputs:
 *  - imgCounter (int)
 *  - videoSequence (std::string)
 *  - imgL and imgR (cv::Mat, left and right image)
 *  - section (cv::Mat, labeled accessible section ground thruth as binary image)
 *  - disparity (cv::Mat, disparities calculated using A. Geigers libELAS)
 * Configuration Options: please see FlowerBoxReader.conf
 */
class FlowerBoxReader : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		FlowerBoxReader(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~FlowerBoxReader();

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

		// outputs
		BVS::Connector<int> imgCounter; /**< Sequence counter. */
		BVS::Connector<std::string> videoSequence; /**< Video sequence name. */
		BVS::Connector<cv::Mat> imgL; /**< Left image output. */
		BVS::Connector<cv::Mat> imgR; /**< Right image output. */
		BVS::Connector<cv::Mat> section; /**< Accessible section as binary image. */
		BVS::Connector<cv::Mat> disparity; /**< Precalculated disparity map. */

		// settings
		std::string dataDir; /**< FlowerBox dataset base directory. */
		std::vector<std::string> videoList; /**< List of sequence to load. */

		// variables
		std::string video; /**< Current video sequence read from. */
		int counter; /**< Sequence counter, part of filename to load from. */
		bool requestShutdown = false; /**< Used to signal incorrect settings. */

		/** Assemble filepath and name.
		 * Assemble the filename and its path from given input for a dataset
		 * image, like: data/flowerbox/img/alley/frame_00030.png.
		 * @param[in] dataDir Dataset base directory.
		 * @param[in] counter Sequence counter.
		 * @param[in] video Video sequence name.
		 * @param[in] prefix Image/resource location prefix.
		 * @param[in] suffix Image/resource suffix.
		 * @return Filepath and name.
		 */
		std::string assembleFileName(const std::string& dataDir, const int& counter, const std::string& video, const std::string& prefix, const std::string& suffix = {});

		FlowerBoxReader(const FlowerBoxReader&) = delete; /**< -Weffc++ */
		FlowerBoxReader& operator=(const FlowerBoxReader&) = delete; /**< -Weffc++ */
};



#endif //FLOWERBOXREADER_H

