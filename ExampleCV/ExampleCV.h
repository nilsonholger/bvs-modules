#ifndef EXAMPLECV_H
#define EXAMPLECV_H

#include "bvs/module.h"
#include <opencv2/core/core.hpp>



/** This is the ExampleCV module.
 * This is just a simple example module using OpenCV. It receives an input
 * image and then uses OpenCV's capabilites to do a grey conversion, apply some
 * blur and a canny edge detector. Finally, it displays the image.
 *
 * Requires: OpenCV
 * Input: input<cv::Mat>
 * Output: output<cv::Mat>
 * Configuration Options:
 *    convertToGrey -> enable/disable grey conversion
 *    blurSize -> size of blur kernel
 *    cannyThreshold -> edge detector's upper threshold
 *    showResult -> show final result
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

		bool convertToGrey; /**< Whether to convert image to grey. */
		int blurSize; /**< Size of blur kernel. */
		int cannyThreshold; /**< Upper threshold for canny edge detector. */
		bool showResult; /** Display final result. */
		cv::Mat img; /**< Local storage for input image. */
		cv::Mat tmpImg; /**< Local temp storage. */

		ExampleCV(const ExampleCV&) = delete; /**< -Weffc++ */
		ExampleCV& operator=(const ExampleCV&) = delete; /**< -Weffc++ */
};



#endif //EXAMPLECV_H

