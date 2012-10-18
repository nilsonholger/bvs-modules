#ifndef CAPTURECV_H
#define CAPTURECV_H

#include "bvs/module.h"
#include "opencv2/opencv.hpp"
#include <vector>



/** This is the CaptureCV class.
 * Please add sufficient documentation to enable others to use it.
 * Include information about:
 * - Dependencies
 * - Inputs
 * - Outputs
 * - Configuration Options
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
		std::vector<BVS::Connector<cv::Mat>*> outputs;

		std::vector<cv::VideoCapture> captures;
		int numInputs;

		CaptureCV(const CaptureCV&) = delete; /**< -Weffc++ */
		CaptureCV& operator=(const CaptureCV&) = delete; /**< -Weffc++ */
};

#endif //CAPTURECV_H

