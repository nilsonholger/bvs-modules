#include "CaptureCV.h"



CaptureCV::CaptureCV(const std::string id, const BVS::Info& bvs)
	: BVS::Module(),
	id(id),
	logger(id),
	bvs(bvs),
	outputs(),
	captures(),
	numInputs(bvs.config.getValue<int>(id+".numInputs", 0))
{
	if (numInputs==0)
	{
		LOG(1, "invalid number of inputs selected, please set 'numInputs'>0 in your config!");
		exit(1);
	}

	for (int i=0; i<numInputs; i++)
	{
		captures.emplace_back(cv::VideoCapture(i));
		// for firewire grasshoppers:
		// 0 normal
		// 1
		// 2 black'n'white
		// 3 weird black'n'white crop
		//captures[i].open(i);
		captures[i].set(CV_CAP_PROP_MODE, 2);
		outputs.emplace_back(new BVS::Connector<cv::Mat>("out"+std::to_string(i+1), BVS::ConnectorType::OUTPUT));

		if (!captures[i].isOpened())
		{
			LOG(0, "Could not open cameras!");
			exit(1);
		}
	}
}



CaptureCV::~CaptureCV()
{
	for (auto cap: captures) cap.release();
	for (auto out: outputs) delete out;
}



BVS::Status CaptureCV::execute()
{
	for (auto out: outputs) out->lockConnection();
	for (auto cap: captures) cap.grab();
	for (int i=0; i<numInputs; i++) captures.at(i).retrieve(**outputs.at(i));
	for (auto out: outputs) out->unlockConnection();

	return BVS::Status::OK;
}



// UNUSED
BVS::Status CaptureCV::debugDisplay()
{
	return BVS::Status::OK;
}



extern "C" {
	int bvsRegisterModule(std::string id, const BVS::Info& bvs)
	{
		registerModule(id, new CaptureCV(id, bvs));

		return 0;
	}
}

