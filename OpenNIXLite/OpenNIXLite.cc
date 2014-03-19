#include "OpenNIXLite.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
OpenNIXLite::OpenNIXLite(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(bvs)
	//, input("inputName", BVS::ConnectorType::INPUT)
	, outColor("outColor", BVS::ConnectorType::OUTPUT)
	, outDepth("outDepth", BVS::ConnectorType::OUTPUT)

	// CONFIGURATION RETRIEVAL
	//, yourSwitch(bvs.config.getValue<bool>(info.conf + ".yourSwitch", false))
{

}



// This is your module's destructor.
OpenNIXLite::~OpenNIXLite()
{

}



// Put all your work here.
BVS::Status OpenNIXLite::execute()
{
	// LOGGING: use the LOG(...) macro
	//LOG(3, "Execution of " << info.id << "!");

	// VARIOUS INFORMATION FROM BVS
	//unsigned long long round = bvs.round;
	//int lastRoundModuleDuration = bvs.moduleDurations.find(info.id)->second.count();
	//int lastRoundDuration = bvs.lastRoundDuration.count();

	// CONNECTOR USAGE: it is always a good idea to check input, twice
	//int incoming;
	//if (!input.receive(incoming)) return BVS::Status::NOINPUT;
	//if (incoming==int()) return BVS::Status::NOINPUT;
	//std::string message = "received " + std::to_string(incoming);
	//output.send(message);

	opennilite.colorCB([&](cv::Mat3b color){outColor.send(color);});
	opennilite.depthCB([&](cv::Mat1s depth){outDepth.send(depth);});

	return BVS::Status::OK;
}



// UNUSED
BVS::Status OpenNIXLite::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(OpenNIXLite)

