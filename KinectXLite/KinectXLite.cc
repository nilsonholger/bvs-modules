#include "KinectXLite.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to configuration data.
KinectXLite::KinectXLite(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module(),
	info(info),
	logger(info.id),
	bvs(bvs),
	//input("inputName", BVS::ConnectorType::INPUT),
	kinect(),
	disp2Depth(),
	dispVec(),
	depth(480, 640, CV_16UC1),
	disp(480, 640, CV_16UC1),
	baseline(bvs.config.getValue<double>(info.conf+".baseline", 0.06)),
	focalLength(bvs.config.getValue<int>(info.conf+".focalLength", 740)),
	requestShutdown(false),
	outDepth("outDepth", BVS::ConnectorType::OUTPUT),
	outDisp("outDisp", BVS::ConnectorType::OUTPUT)
{
	if (!kinect.isOpen())
	{
		LOG(1, "could not connect to Kinect device, aborting now!");
		requestShutdown = true;
	}

	kinect.startDepth();
	disp2Depth = kinect.getDefaultDisp2Depth();
	disp2Depth[2047]=0; // Key for empty areas
}



// This is your module's destructor.
KinectXLite::~KinectXLite()
{

}



// Put all your work here.
BVS::Status KinectXLite::execute()
{
	if (requestShutdown) return BVS::Status::SHUTDOWN;

	kinect.getDepth(dispVec);

	for (int i=0; i<depth.rows; i++)
		for (int j=0; j<depth.cols; j++) {
			disp.at<uint16_t>(i,j) = focalLength * baseline / disp2Depth[dispVec[i*640+j]];
			depth.at<uint16_t>(i,j) = disp2Depth[dispVec[i*640+j]];
		}

	outDepth.send(depth);
	outDisp.send(disp);

	return BVS::Status::OK;
}



// UNUSED
BVS::Status KinectXLite::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(KinectXLite)

