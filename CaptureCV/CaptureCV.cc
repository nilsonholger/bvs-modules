#include "CaptureCV.h"



CaptureCV::CaptureCV(const std::string id, const BVS::Info& bvs)
	: BVS::Module(),
	id(id),
	logger(id),
	bvs(bvs),
	outputs(),
	inputs(),
	captures(),
	writers(),
	numNodes(bvs.config.getValue<int>(id+".numNodes", 1)),
	fileList(),
	useFile(bvs.config.getValue<bool>(id+".useFile", false)),
	captureMode(bvs.config.getValue<int>(id+".captureMode", -1)),
	captureFPS(bvs.config.getValue<double>(id+".captureFPS", -1.0)),
	recordVideo(bvs.config.getValue<bool>(id+".recordVideo", false)),
	recordFOURCC(bvs.config.getValue<std::string>(id+".recordFOURCC", "MJPG")),
	recordFPS(bvs.config.getValue<double>(id+".recordFPS", 0.0)),
	recordWidth(bvs.config.getValue<int>(id+".recordWidth", 0)),
	recordHeight(bvs.config.getValue<int>(id+".recordHeight", 0)),
	recordColor(bvs.config.getValue<int>(id+".recordColor", true)),
	fourcc()
{
	if (useFile || recordVideo)
	{
		bvs.config.getValue<std::string>(id+".fileList", fileList);
		if ((int)fileList.size()<numNodes) LOG(0, "Not enough files given to read/write from/to!");
	}

	if (recordVideo)
	{
		if (recordFOURCC.length()!=4) LOG(0, "RecordFOURCC length must be 4!");
		fourcc = CV_FOURCC(recordFOURCC[0], recordFOURCC[1], recordFOURCC[2], recordFOURCC[3]);
		for (int i=0; i<numNodes; i++)
		{
			writers.emplace_back(cv::VideoWriter());
			inputs.emplace_back(new BVS::Connector<cv::Mat>("in"+std::to_string(i+1), BVS::ConnectorType::INPUT));
		}
	}
	else
	{
		for (int i=0; i<numNodes; i++)
		{
			if (useFile)
			{
				captures[i].open(fileList.at(i));
			}
			else
			{
				captures.emplace_back(cv::VideoCapture(i));
				if (captureMode>=0) captures[i].set(CV_CAP_PROP_MODE, captureMode);
				if (captureFPS>=0) captures[i].set(CV_CAP_PROP_FPS, captureFPS);
				if (!captures[i].isOpened()) LOG(0, "Could not open camera: " << i << "!");
			}
			outputs.emplace_back(new BVS::Connector<cv::Mat>("out"+std::to_string(i+1), BVS::ConnectorType::OUTPUT));
		}
	}
}



CaptureCV::~CaptureCV()
{
	if (recordVideo)
	{
		for (auto wr: writers) wr.release();
		for (auto in: inputs) delete in;
	}
	else
	{
		for (auto cap: captures) cap.release();
		for (auto out: outputs) delete out;
	}
}



BVS::Status CaptureCV::execute()
{
	if (recordVideo)
	{
		for (auto in: inputs) in->lockConnection();
		bool empty = false;
		for (auto in: inputs) if ((*in)->empty()) empty = true;
		if (!empty)
		{
			LOG(2, "Writing frame(s) to " << numNodes << " file(s)!");
			for (int i=0; i<numNodes; i++)
			{
				if (!writers[i].isOpened())
				{
					if (recordWidth==0) recordWidth = (**inputs[i]).cols;
					if (recordHeight==0) recordHeight = (**inputs[i]).rows;
					writers[i].open(fileList[i], fourcc, recordFPS, cv::Size(recordWidth, recordHeight), recordColor);
					if (!writers[i].isOpened()) LOG(0, "Could not open writer for '" << fileList[i]);
				}
				writers[i].write(**inputs[i]);
			}
		}
		for (auto in: inputs) in->unlockConnection();
	}
	else
	{
		for (auto out: outputs) out->lockConnection();
		for (auto cap: captures) cap.grab();
		for (int i=0; i<numNodes; i++) captures[i].retrieve(**outputs[i]);
		for (auto out: outputs) out->unlockConnection();
	}

	return BVS::Status::OK;
}



// UNUSED
BVS::Status CaptureCV::debugDisplay()
{
	return BVS::Status::OK;
}

