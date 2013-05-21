#include "ExampleCV.h"
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
	mode(bvs.config.getValue<std::string>(info.conf+".mode", "S").at(0)),
	img(),
	tmpImg(),
	capture()

	// CONFIGURATION RETRIEVAL
	//yourSwitch(bvs.config.getValue<bool>(info.conf + ".yourSwitch", false)),
{
	if (mode == 'C')
	{
		capture.open(0);
		capture.set( CV_CAP_PROP_FRAME_WIDTH,640  );
		capture.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );
	}
	
}



// This is your module's destructor.
ExampleCV::~ExampleCV()
{

}



// Put all your work here.
BVS::Status ExampleCV::execute()
{
	// CAPTURE
	if (mode == 'C') {
		capture >> tmpImg;
	} else {
		// CONNECTOR USAGE: it is always a good idea to check input, twice
		if (!input.receive(img)) return BVS::Status::NOINPUT;
		if (img.empty()) return BVS::Status::NOINPUT;

		switch (mode) {
			case 'G':
				// GREY CONVERSION
				cv::cvtColor(img, tmpImg, CV_BGR2GRAY);
				break;
			case 'B':
				// BLUR APPLIANCE
				cv::GaussianBlur(img, tmpImg, cv::Size(7,7), 1.5, 1.5);
				break;
			case 'E':
				// CANNY EDGE DETECTION
				cv::Canny(img, tmpImg, 0, 30, 3);
				break;
			case 'S':
				// SHOW IMAGE USING OPENCV
				cv::putText(img, bvs.getFPS(), cv::Point(10, 30),
						CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(255, 255, 255), 2);
				cv::imshow("blur", img);
				cv::waitKey(1);
				break;
		}
	}

	output.send(tmpImg);

	return BVS::Status::OK;
}



// UNUSED
BVS::Status ExampleCV::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(ExampleCV)

