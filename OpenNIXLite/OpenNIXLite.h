#ifndef OPENNIXLITE_H
#define OPENNIXLITE_H

#include "bvs/module.h"
#include "opencv2/opencv.hpp"

// Many thanks to Manel.
#include "opennilite.hpp"



/** This is the OpenNIXLite module.
 * Please add sufficient documentation to enable others to use it.
 * Include information about: Dependencies, Inputs, Outputs, Configuration Options...
 */
class OpenNIXLite : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		OpenNIXLite(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~OpenNIXLite();

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

		/** Example Connector used to retrieve/send data from/to other modules.
		 * @see Connector
		 */
		//BVS::Connector<int> input;
		BVS::Connector<cv::Mat> output;

		OpenNILite opennilite;

		OpenNIXLite(const OpenNIXLite&) = delete; /**< -Weffc++ */
		OpenNIXLite& operator=(const OpenNIXLite&) = delete; /**< -Weffc++ */
};



#endif //OPENNIXLITE_H

