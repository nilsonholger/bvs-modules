#ifndef BVSCAPTURE_H
#define BVSCAPTURE_H

#include "bvs/module.h"

#include "opencv2/opencv.hpp"



/** This is the bvsCapture class.
 * Please add sufficient documentation to enable others to use it.
 * Include information about:
 * - Dependencies
 * - Inputs
 * - Outputs
 * - Configuration Options
 */
class bvsCapture : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] id Your modules unique identifier, will be set by framework.
		 * @param[in] config Reference to config to enable option retrieval.
		 */
		bvsCapture(const std::string id, const BVS::Info& bvs);

		/** Your module destructor. */
		~bvsCapture();

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
		BVS::Connector<cv::Mat> outL;
		BVS::Connector<cv::Mat> outR;

		cv::VideoCapture captureL;
		cv::VideoCapture captureR;

		bvsCapture(const bvsCapture&) = delete; /**< -Weffc++ */
		bvsCapture& operator=(const bvsCapture&) = delete; /**< -Weffc++ */
};

#endif //BVSCAPTURE_H

