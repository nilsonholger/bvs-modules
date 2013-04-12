#include "ExampleCV.h"



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
	tmpImg()

	// CONFIGURATION RETRIEVAL
	//yourSwitch(bvs.config.getValue<bool>(info.conf + ".yourSwitch", false)),
{

}



// This is your module's destructor.
ExampleCV::~ExampleCV()
{

}



// Put all your work here.
BVS::Status ExampleCV::execute()
{
	// LOGGING: use the LOG(...) macro
	//LOG(3, "Execution of " << info.id << "!");

	// VARIOUS INFORMATION FROM BVS
	//unsigned long long round = bvs.round;
	//int lastRoundModuleDuration = bvs.moduleDurations.find(info.id)->second.count();
	//int lastRoundDuration = bvs.lastRoundDuration.count();

	// CONNECTOR USAGE: it is always a good idea to check input, twice
	if (!input.receive(img)) return BVS::Status::NOINPUT;
	if (img.empty()) return BVS::Status::NOINPUT;

	switch (mode) {

		// GREY CONVERSION
		case 'G':
			cv::cvtColor(img, tmpImg, CV_BGR2GRAY);
			break;

		// BLUR APPLIANCE
		case 'B':
			cv::GaussianBlur(img, tmpImg, cv::Size(7,7), 1.5, 1.5);
			break;

		// CANNY EDGE DETECTION
		case 'C':
			cv::Canny(img, tmpImg, 0, 30, 3);
			break;

		// SHOW IMAGE USING OPENCV
		case 'S':
			cv::putText(img, bvs.getFPS(), cv::Point(10, 30),
					CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(255, 255, 255), 2);
			cv::imshow("blur", img);
			cv::waitKey(1);
			break;
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

