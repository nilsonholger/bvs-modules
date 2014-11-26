#include "FlowerBoxReader.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
FlowerBoxReader::FlowerBoxReader(BVS::ModuleInfo info, const BVS::Info& _bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(_bvs)
	, imgCounter("imgCounter", BVS::ConnectorType::OUTPUT)
	, videoSequence("videoSequence", BVS::ConnectorType::OUTPUT)
	, imgL("imgL", BVS::ConnectorType::OUTPUT)
	, imgR("imgR", BVS::ConnectorType::OUTPUT)
	, section("section", BVS::ConnectorType::OUTPUT)
	, disparity("disparity", BVS::ConnectorType::OUTPUT)
	, dataDir(bvs.config.getValue<std::string>(info.conf+".directory", {}))
	, videoList{{}}
	, video{}
	, counter{30}
{
	if (dataDir.empty()) requestShutdown = true;
	bvs.config.getValue(info.conf+".videoList", videoList);
	if (videoList.empty()) requestShutdown = true;
	video = videoList.front();
	videoList.erase(videoList.begin());
}



// This is your module's destructor.
FlowerBoxReader::~FlowerBoxReader()
{

}



// Put all your work here.
BVS::Status FlowerBoxReader::execute()
{
	if (requestShutdown) return BVS::Status::SHUTDOWN;

	// load left image, if necessary, advance to next sequence
	std::string path = assembleFileName(dataDir, counter, video, "img");
	cv::Mat tmpL = cv::imread(path, -1);
	if (tmpL.empty()) {
		if (videoList.empty()) {
			return BVS::Status::SHUTDOWN;
		} else {
			video = videoList.front();
			videoList.erase(videoList.begin());
			counter = 30;
			path = assembleFileName(dataDir, counter, video, "img");
			tmpL = cv::imread(path, -1);
		}
	}

	// send left image and meta data
	imgCounter.send(counter);
	videoSequence.send(video);
	if (tmpL.empty()) {
		LOG(0, "Error, could not load: " << path);
		return BVS::Status::SHUTDOWN;
	}
	imgL.send(tmpL);

	// send right image
	if (imgR.active()) {
		path = assembleFileName(dataDir, counter, video, "img", "R");
		cv::Mat tmpR = cv::imread(path, -1);
		if (tmpR.empty()) {
			LOG(0, "Error, could not load: " << path);
			return BVS::Status::SHUTDOWN;
		}
		imgR.send(tmpR);
	}

	// send accessible section ground truth
	if (section.active()) {
		path = assembleFileName(dataDir, counter, video, "meta/section");
		cv::Mat tmpSection = cv::imread(path, -1);
		if (tmpSection.empty()) {
			LOG(0, "Error, could not load: " << path);
			return BVS::Status::SHUTDOWN;
		}
		section.send(tmpSection);
	}

	// send precalculated disparity map
	if (disparity.active()) {
		path = assembleFileName(dataDir, counter, video, "meta/disparities");
		cv::Mat tmpDisparity = cv::imread(path, -1);
		if (tmpDisparity.empty()) {
			LOG(0, "Error, could not load: " << path);
			return BVS::Status::SHUTDOWN;
		}
		disparity.send(tmpDisparity);
	}

	// advance to next image
	counter += 5;

	return BVS::Status::OK;
}



std::string FlowerBoxReader::assembleFileName(const std::string& dataDir, const int& counter, const std::string& video, const std::string& prefix, const std::string& suffix)
{
	std::string tmp = std::to_string(counter);
	tmp.insert(0, 5-tmp.length(), '0');
	std::string file = dataDir + '/' + prefix + '/' + video + "/frame_" + tmp + suffix + ".png";
	return file;
}



// UNUSED
BVS::Status FlowerBoxReader::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(FlowerBoxReader)

