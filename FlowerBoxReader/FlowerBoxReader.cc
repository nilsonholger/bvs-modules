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
	, imgL("imgL", BVS::ConnectorType::OUTPUT)
	, imgR("imgR", BVS::ConnectorType::OUTPUT)
	, imgName("imgName", BVS::ConnectorType::OUTPUT)
	, videoSequence("videoSequence", BVS::ConnectorType::OUTPUT)
	, filename()
	, directory(bvs.config.getValue<std::string>(info.conf+".directory", {}))
	, videoList{{}}
{
	if (directory.empty()) requestShutdown = true;
	bvs.config.getValue(info.conf+".videoList", videoList);
	if (videoList.empty()) requestShutdown = true;
}



// This is your module's destructor.
FlowerBoxReader::~FlowerBoxReader()
{

}



// Put all your work here.
BVS::Status FlowerBoxReader::execute()
{
	if (requestShutdown) return BVS::Status::SHUTDOWN;

	// for every entry in videoList, check directory, load files:
	//   filename: directory + 'img/' + video + base + counter (starting at initial offset: 30)
	//   increase by stepsize: 5
	//   until file not found -> advance to next video
	// until all videoList entries processed
	// request Shutdown

	return BVS::Status::OK;
}



// UNUSED
BVS::Status FlowerBoxReader::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(FlowerBoxReader)

