#include "ExampleCV.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
ExampleCV::ExampleCV(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module(),
	info(info),
	logger(info.id),
	bvs(bvs),
	input("input", BVS::ConnectorType::INPUT),
	output("output", BVS::ConnectorType::OUTPUT),
	convertToGrey(bvs.config.getValue<bool>(info.conf+".convertToGrey", true)),
	blurSize(bvs.config.getValue<int>(info.conf+".blurSize", 7)),
	cannyThreshold(bvs.config.getValue<int>(info.conf+".cannyThreshold", 30)),
	showResult(bvs.config.getValue<bool>(info.conf+".showResult", true)),
	img(),
	tmpImg()

	// CONFIGURATION RETRIEVAL
	//yourSwitch(bvs.config.getValue<bool>(info.conf + ".yourSwitch", false)),
{
	if (showResult)
	{
		cv::namedWindow("result");
		cv::startWindowThread();
	}
}



// This is your module's destructor.
ExampleCV::~ExampleCV()
{

}



// Put all your work here.
BVS::Status ExampleCV::execute()
{
	// get input image
	// connector usage: it is always a good idea to check input
	if (!input.receive(img)) return BVS::Status::NOINPUT;
	if (img.empty()) return BVS::Status::NOINPUT;

	// grey conversion
	if (convertToGrey) cv::cvtColor(img, tmpImg, CV_BGR2GRAY);
	img = tmpImg;

	// blur appliance
	cv::GaussianBlur(img, tmpImg, cv::Size(blurSize, blurSize), 1.5, 1.5);
	img = tmpImg;

	// canny edge detection
	cv::Canny(img, tmpImg, 0, cannyThreshold, 3);
	img = tmpImg;

	// send to other modules
	output.send(tmpImg);

	// show image using opencv
	if (showResult)
	{
		cv::putText(img, bvs.getFPS(), cv::Point(10, 30), CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(255, 255, 255), 2);
		cv::imshow("result", img);
		cv::waitKey(1);
	}

	return BVS::Status::OK;
}



// UNUSED
BVS::Status ExampleCV::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(ExampleCV)

