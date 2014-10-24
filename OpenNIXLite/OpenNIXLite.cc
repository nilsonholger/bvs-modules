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
	, mutex()
	, lock(mutex, std::defer_lock)
	, color()
	, depth()

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
	opennilite.colorCB([&](cv::Mat3b _color){
			std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
			lock.lock();
			color=_color;
			lock.unlock();
			});
	opennilite.depthCB([&](cv::Mat1s _depth){
			std::unique_lock<std::mutex> lock(mutex, std::defer_lock);
			lock.lock();
			depth=_depth;
			lock.unlock();
			});

	lock.lock();
	outColor.send(color);
	outDepth.send(depth);
	lock.unlock();

	return BVS::Status::OK;
}



// UNUSED
BVS::Status OpenNIXLite::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(OpenNIXLite)

