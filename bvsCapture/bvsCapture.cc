#include "bvsCapture.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to the its config.
// However, you should use it to create your data structures etc.
bvsCapture::bvsCapture(const std::string id, const BVS::Info& bvs)
	: BVS::Module()
	, id(id)
	, logger(id)
	, bvs(bvs)
	, outL("outL", BVS::ConnectorType::OUTPUT)
	, outR("outR", BVS::ConnectorType::OUTPUT)
	, captureL()
	, captureR()
{
	captureL.open(0);
	captureR.open(1);

	// for firewire grasshoppers:
	// 0 normal
	// 1
	// 2 black'n'white
	// 3 weird black'n'white crop
	captureL.set(CV_CAP_PROP_MODE, 0);
	captureR.set(CV_CAP_PROP_MODE, 0);

	//if (!captureL.isOpened() || !captureR.isOpened())
	//{
	//	LOG(0, "Could not open cameras!");
	//	exit(-1);
	//}
}



// This is your module's destructor.
// See the constructor for more info.
bvsCapture::~bvsCapture()
{
	captureL.release();
	captureR.release();

}



// Put all your work here.
BVS::Status bvsCapture::execute()
{
	LOG(2, "Execution of " << id << "!");

	outL.lockConnection();
	outR.lockConnection();
	captureL.grab();
	captureR.grab();
	captureL.retrieve(*outL);
	captureR.retrieve(*outR);
	outL.unlockConnection();
	outR.unlockConnection();

	return BVS::Status::OK;
}



// UNUSED
BVS::Status bvsCapture::debugDisplay()
{
	return BVS::Status::OK;
}



// This function is called by the framework upon creating a module instance of
// this class. It creates the module and registers it within the framework.
// DO NOT CHANGE OR DELETE
extern "C" {
	// register with framework
	int bvsRegisterModule(std::string id, const BVS::Info& bvs)
	{
		registerModule(id, new bvsCapture(id, bvs));

		return 0;
	}
}

