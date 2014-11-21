#ifndef FLOWERBOXREADER_H
#define FLOWERBOXREADER_H

#include<string>
#include<vector>

#include "bvs/module.h"
#include "opencv2/core/core.hpp"



/** This is the FlowerBoxReader module.
 * Please add sufficient documentation to enable others to use it.
 * Include information about: Dependencies, Inputs, Outputs, Configuration Options...
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

		BVS::Connector<cv::Mat> imgL;
		BVS::Connector<cv::Mat> imgR;
		BVS::Connector<std::string> imgName;
		BVS::Connector<std::string> videoSequence;

		std::string filename;
		std::string directory;
		std::vector<std::string> videoList;

		bool requestShutdown = false;

		FlowerBoxReader(const FlowerBoxReader&) = delete; /**< -Weffc++ */
		FlowerBoxReader& operator=(const FlowerBoxReader&) = delete; /**< -Weffc++ */
};



#endif //FLOWERBOXREADER_H

