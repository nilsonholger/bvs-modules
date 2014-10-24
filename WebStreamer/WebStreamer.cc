#include "WebStreamer.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
WebStreamer::WebStreamer(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(bvs)
	, input("input", BVS::ConnectorType::INPUT)
	//, output("outputName", BVS::ConnectorType::OUTPUT)
	, mjpeg(80, 95)
	, tmp()

	// CONFIGURATION RETRIEVAL
	//, yourSwitch(bvs.config.getValue<bool>(info.conf + ".yourSwitch", false))
{

}



// This is your module's destructor.
WebStreamer::~WebStreamer()
{

}



// Put all your work here.
BVS::Status WebStreamer::execute()
{
	if (!input.receive(tmp)) return BVS::Status::NOINPUT;
	if (tmp.empty()) return BVS::Status::NOINPUT;
	
	input.lockConnection();
	mjpeg.add(tmp);
	input.unlockConnection();

	return BVS::Status::OK;
}



// UNUSED
BVS::Status WebStreamer::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(WebStreamer)

