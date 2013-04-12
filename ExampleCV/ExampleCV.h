#ifndef EXAMPLECV_H
#define EXAMPLECV_H

#include "bvs/module.h"
#include "opencv2/opencv.hpp"



/** This is the ExampleCV module.
 * This is just a simple example module using OpenCV.
 * While it combines the capabilities to do grey conversion, blur, canny edge
 * detection and display of an image, it is inteded to show the usage of module
 * with the BVS framework. See the provided example config for more
 * information.
 *
 * Requires: OpenCV
 * Inputs: input<cv::Mat>
 * Outputs: output<cv::Mat>
 * Configuration Options:
 *    mode -> can be GREY, BLUR, CANNY or SHOW
 */
class ExampleCV : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		ExampleCV(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~ExampleCV();

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
		BVS::Connector<cv::Mat> input;
		BVS::Connector<cv::Mat> output;

		char mode; /**< Example module mode to use (grey, blur, canny, show). */
		cv::Mat img; /**< Local storage for input image. */
		cv::Mat tmpImg; /**< Local temp storage. */

		ExampleCV(const ExampleCV&) = delete; /**< -Weffc++ */
		ExampleCV& operator=(const ExampleCV&) = delete; /**< -Weffc++ */
};



#endif //EXAMPLECV_H

