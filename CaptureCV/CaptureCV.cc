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
	numNodes(bvs.config.getValue<int>(id+".numNodes", 0)),
	mode(bvs.config.getValue<std::string>(id+".mode", "C").at(0)),
	videoFiles(),
	imageFiles(bvs.config.getValue<std::string>(id+".imageFiles", "images/frame_{FRAME}_{NODE}.jpg")),
	fileNamePieces(),
	imageCounter(1),
	cameraMode(bvs.config.getValue<int>(id+".captureMode", -1)),
	cameraFPS(bvs.config.getValue<double>(id+".captureFPS", -1.0)),
	recordFOURCC(bvs.config.getValue<std::string>(id+".recordFOURCC", "MJPG")),
	fourcc(),
	recordFPS(bvs.config.getValue<double>(id+".recordFPS", 0.0)),
	recordWidth(bvs.config.getValue<int>(id+".recordWidth", 0)),
	recordHeight(bvs.config.getValue<int>(id+".recordHeight", 0)),
	recordColor(bvs.config.getValue<int>(id+".recordColor", true)),
	requestShutdown(false)
{
	if (numNodes==0)
	{
		LOG(0, "Number of nodes not set!");
		requestShutdown = true;
	}

	switch (mode)
	{
		case 'C': case 'V': case 'I':
			for (int i=0; i<numNodes; i++)
				outputs.emplace_back(new BVS::Connector<cv::Mat>("out"+std::to_string(i+1), BVS::ConnectorType::OUTPUT));
			break;
		case 'R': case 'S':
			for (int i=0; i<numNodes; i++)
				inputs.emplace_back(new BVS::Connector<cv::Mat>("in"+std::to_string(i+1), BVS::ConnectorType::INPUT));
			break;
	}

	if (mode=='V' || mode=='R')
	{
		bvs.config.getValue<std::string>(id+".videoFiles", videoFiles);
		if ((int)videoFiles.size()<numNodes)
		{
			LOG(0, "Insufficient number of video files!");
			requestShutdown = true;
		}
	}

	switch (mode)
	{
		case 'C':
			for (int i=0; i<numNodes; i++)
			{
				captures.emplace_back(cv::VideoCapture(i));
				if (cameraMode>=0) captures.at(i).set(CV_CAP_PROP_MODE, cameraMode);
				if (cameraFPS>=0) captures.at(i).set(CV_CAP_PROP_FPS, cameraFPS);
				if (!captures.at(i).isOpened())
				{
					LOG(0, "Could not open camera: " << i << "!");
					requestShutdown = true;
				}
			}
			break;
		case 'V':
			for (int i=0; i<numNodes; i++)
			{
				captures.emplace_back(cv::VideoCapture(videoFiles.at(i)));
				LOG(2, "Use file '" << videoFiles.at(i) << "' as source for node: " << i+1);
			}
			break;
		case 'I': parseImageFileName(); break;
		case 'R':
			if (recordFOURCC.length()!=4) { LOG(0, "RecordFOURCC length must be 4!"); }
			else { fourcc = CV_FOURCC(recordFOURCC[0], recordFOURCC[1], recordFOURCC[2], recordFOURCC[3]); }
			for (int i=0; i<numNodes; i++) writers.emplace_back(cv::VideoWriter());
			break;
		case 'S': parseImageFileName(); break;
		default:
			LOG(0, "Incorrect mode: '" << mode << "', aborting!");
			requestShutdown = true;
			break;
	}
}



CaptureCV::~CaptureCV()
{
	switch (mode)
	{
		case 'C': case 'V':
			for (auto cap: captures) if (cap.isOpened()) cap.release();
			break;
		case 'I': break;
		case 'R': for (auto wr: writers) if (wr.isOpened()) wr.release();
			break;
		case 'S': break;
		default: break;
	}

	for (auto& in: inputs) delete in;
	for (auto& out: outputs) delete out;
}



BVS::Status CaptureCV::execute()
{
	if (requestShutdown) return BVS::Status::SHUTDOWN;

	for (auto out: outputs) out->lockConnection();
	for (auto in: inputs) in->lockConnection();
	switch (mode)
	{
		case 'C':
			for (auto cap: captures) cap.grab();
			for (int i=0; i<numNodes; i++) captures.at(i).retrieve(**outputs.at(i));
			break;
		case 'V':
			for (int i=0; i<numNodes; i++) if (!captures.at(i).read(**outputs.at(i))) requestShutdown = true;
			break;
		case 'I':
			for (int i=0; i<numNodes; i++)
			{
				std::string filename = getImageFileName(imageCounter, i);
				LOG(3, "loading: " << filename);
				cv::Mat tmp = cv::imread(filename);
				if (tmp.empty())
				{
					LOG(0, "cannot open file: " << filename);
					requestShutdown = true;
				}
				**outputs.at(i) = tmp;
			}
			imageCounter++;
			break;
		case 'R':
			for (auto in: inputs)
			{
				cv::Mat tmp;
				if (!in->receive(tmp)) requestShutdown = true;
			}
			if (!requestShutdown)
			{
				LOG(2, "Writing frame(s) to " << numNodes << " file(s)!");
				for (int i=0; i<numNodes; i++)
				{
					if (!writers.at(i).isOpened())
					{
						if (recordWidth==0) recordWidth = (**inputs.at(i)).cols;
						if (recordHeight==0) recordHeight = (**inputs.at(i)).rows;
						writers.at(i).open(videoFiles.at(i), fourcc, recordFPS, cv::Size(recordWidth, recordHeight), recordColor);
						if (!writers.at(i).isOpened()) LOG(0, "Could not open writer for '" << videoFiles.at(i));
					}
					if (!(**inputs.at(i)).empty()) writers.at(i).write(**inputs.at(i));
				}
			}
			break;
		case 'S':
			for (int i=0; i<numNodes; i++)
			{
				std::string filename = getImageFileName(imageCounter, i);
				bool written = cv::imwrite(filename, **inputs.at(i));
				if (!written)
				{
					LOG(0, "Could not wirte to file '" << filename << "'!");
					requestShutdown = true;
				}
			}
			imageCounter++;
			break;
		default: break;
	}
	for (auto in: inputs) in->unlockConnection();
	for (auto out: outputs) out->unlockConnection();

	if (requestShutdown) return BVS::Status::SHUTDOWN;
	return BVS::Status::OK;
}



BVS::Status CaptureCV::debugDisplay()
{
	return BVS::Status::OK;
}



void CaptureCV::parseImageFileName()
{
	std::string tmp = imageFiles;
	while (!tmp.empty())
	{
		size_t begin = tmp.find_first_of('{');
		size_t end = tmp.find_first_of('}');

		if (begin != 0 && begin != end) fileNamePieces.push_back(tmp.substr(0, begin));
		if (begin<end) fileNamePieces.push_back(tmp.substr(begin+1, end-begin-1));
		if (begin == std::string::npos || end == std::string::npos) fileNamePieces.push_back(tmp);

		if (end == std::string::npos) tmp.clear();
		else tmp.erase(0, end+1);
	}
}



std::string CaptureCV::getImageFileName(int frame, int nodeID)
{
	std::string tmp;
	for (auto& piece: fileNamePieces)
	{
		if (piece == "FRAME") tmp += std::to_string(frame);
		else if (piece == "NODE") tmp += std::to_string(nodeID+1);
		else tmp += piece;
	}

	return tmp;
}

