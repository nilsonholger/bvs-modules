#include "CaptureCV.h"
#include <functional>



CaptureCV::CaptureCV(BVS::ModuleInfo info, const BVS::Info& _bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(_bvs)
	, outputs()
	, inputs()
	, folder("folder", BVS::ConnectorType::INPUT)
	, captures()
	, writers()
	, numNodes(bvs.config.getValue<int>(info.conf+".numNodes", 0))
	, nodeOffset(bvs.config.getValue<int>(info.conf+".nodeOffset", 0))
	, mode(bvs.config.getValue<std::string>(info.conf+".mode", "C").at(0))
	, displayMode(bvs.config.getValue<bool>(info.conf+".displayMode", false))
	, videoFiles()
	, imageFiles(bvs.config.getValue<std::string>(info.conf+".imageFiles", "images/frame_{FRAME}_{NODE}.png"))
	, frameNumberPadding(bvs.config.getValue<int>(info.conf+".frameNumberPadding", 5))
	, fileNamePieces()
	, counterStart(bvs.config.getValue<int>(info.conf+".counterStart", 1))
	, stepSize(bvs.config.getValue<int>(info.conf+".stepSize", 1))
	, cameraMode(bvs.config.getValue<int>(info.conf+".cameraMode", -1))
	, cameraFPS(bvs.config.getValue<double>(info.conf+".cameraFPS", -1.0))
	, cameraWidth(bvs.config.getValue<double>(info.conf+".cameraWidth", -1.0))
	, cameraHeight(bvs.config.getValue<double>(info.conf+".cameraHeight", -1.0))
	, recordFOURCC(bvs.config.getValue<std::string>(info.conf+".recordFOURCC", "Y800"))
	, fourcc()
	, recordFPS(bvs.config.getValue<double>(info.conf+".recordFPS", 0.0))
	, recordColor(bvs.config.getValue<bool>(info.conf+".recordColor", true))
{
	if (numNodes==0) LOG(0, "Number of nodes not set!");

	// support functions
	std::function<void()> enableOutputs = [&]() {
		for (int i=0; i<numNodes; i++) {
			outputs.emplace_back(new BVS::Connector<cv::Mat>("out"+std::to_string(i+1), BVS::ConnectorType::OUTPUT));
			if (displayMode) cv::namedWindow("out"+std::to_string(i+1));
		}
		if (displayMode) cv::startWindowThread();
	};

	std::function<void()> enableInputs = [&]() {
		for (int i=0; i<numNodes; i++) {
			inputs.emplace_back(new BVS::Connector<cv::Mat>("in"+std::to_string(i+1), BVS::ConnectorType::INPUT));
			if (displayMode) cv::namedWindow("in"+std::to_string(i+1));
		}
		if (displayMode) cv::startWindowThread();
	};

	std::function<void()> getVideoFiles = [&]() {
		bvs.config.getValue<std::string>(info.conf+".videoFiles", videoFiles);
		if ((int)videoFiles.size()<numNodes) LOG(0, "Insufficient number of video files!");
	};

	std::function<void()> parseImageFileName = [&]() {
		std::string tmp = imageFiles;
		while (!tmp.empty()) {
			size_t begin = tmp.find_first_of('{');
			size_t end = tmp.find_first_of('}');
			if (begin != 0 && begin != end) fileNamePieces.push_back(tmp.substr(0, begin));
			if (begin<end) fileNamePieces.push_back(tmp.substr(begin+1, end-begin-1));
			if (begin == std::string::npos || end == std::string::npos) fileNamePieces.push_back(tmp);
			if (end == std::string::npos) tmp.clear();
			else tmp.erase(0, end+1);
		}
	};

	// init devices/readers/writers
	switch (mode) {
		case 'C':
			enableOutputs();
			for (int i=0; i<numNodes; i++) {
				captures.emplace_back(cv::VideoCapture(i+nodeOffset));
				if (cameraMode>=0) captures.at(i).set(cv::CAP_PROP_MODE, cameraMode);
				if (cameraFPS>=0) captures.at(i).set(cv::CAP_PROP_FPS, cameraFPS);
				if (cameraWidth>=0) captures.at(i).set(cv::CAP_PROP_FRAME_WIDTH, cameraWidth);
				if (cameraHeight>=0) captures.at(i).set(cv::CAP_PROP_FRAME_HEIGHT, cameraHeight);
				if (!captures.at(i).isOpened()) LOG(0, "Could not open camera: " << i << "!");
			}
			break;
		case 'V':
			enableOutputs();
			getVideoFiles();
			for (int i=0; i<numNodes; i++) {
				captures.emplace_back(cv::VideoCapture(videoFiles.at(i)));
				if (captures.at(i).isOpened()) { LOG(2, "Use file '" << videoFiles.at(i) << "' as source for node: " << i+1); }
				else { LOG(0, "Could not open '" << videoFiles.at(i) << "'!"); }
			}
			break;
		case 'I':
			enableOutputs();
			parseImageFileName();
			break;
		case 'R':
			enableInputs();
			getVideoFiles();
			if (recordFOURCC.length()!=4) { LOG(0, "RecordFOURCC length must be 4!"); }
			else { fourcc = cv::VideoWriter::fourcc(recordFOURCC[0], recordFOURCC[1], recordFOURCC[2], recordFOURCC[3]); }
			for (int i=0; i<numNodes; i++) writers.emplace_back(cv::VideoWriter());
			break;
		case 'S':
			enableInputs();
			parseImageFileName();
			break;
		case 'D':
			displayMode = true;
			enableInputs();
			break;
		default:
			LOG(0, "Incorrect mode: '" << mode << "', aborting!");
			break;
	}
}



CaptureCV::~CaptureCV()
{
	switch (mode) {
		case 'C': case 'V': for (auto& cap: captures) if (cap.isOpened()) cap.release(); break;
		case 'I': break;
		case 'R': for (auto& wr: writers) if (wr.isOpened()) wr.release(); break;
		case 'S': break;
		case 'D': break;
		default: break;
	}

	for (auto& in: inputs) delete in;
	for (auto& out: outputs) delete out;
}



BVS::Status CaptureCV::execute()
{
	for (auto& in: inputs) {
		cv::Mat tmp;
		if (!in->receive(tmp)) return BVS::Status::NOINPUT;
		if((**in).empty()) return BVS::Status::NOINPUT;
	}

	for (auto& in: inputs) in->lockConnection();
	for (auto& out: outputs) out->lockConnection();

	switch (mode) {
		case 'C':
			for (auto cap: captures) cap.grab();
			for (int i=0; i<numNodes; i++) {
				captures.at(i).retrieve(**outputs.at(i));
				if (displayMode) cv::imshow("out"+std::to_string(i+1), **outputs.at(i));
			}
			break;
		case 'V':
			for (int i=0; i<numNodes; i++) {
				if (!captures.at(i).read(**outputs.at(i))) LOG(0, "Could not read from '" << videoFiles.at(i) << "'!");
				if (displayMode) cv::imshow("out"+std::to_string(i+1), **outputs.at(i));
			}
			break;
		case 'I':
			for (int i=0; i<numNodes; i++) {
				std::string filename = getImageFileName(counterStart, i);
				LOG(3, "loading: " << filename);
				cv::Mat tmp = cv::imread(filename, -1);
				if (tmp.empty()) LOG(0, "cannot open file: " << filename);
				**outputs.at(i) = tmp;
				if (displayMode) cv::imshow("out"+std::to_string(i+1), **outputs.at(i));
			}
			counterStart += stepSize;
			break;
		case 'R':
			for (int i=0; i<numNodes; i++) {
				if (!writers.at(i).isOpened()) {
					LOG(2, videoFiles.at(i) << ":" << (**inputs.at(i)).cols << "x" << (**inputs.at(i)).rows << "@" << recordFPS << "fps(" << recordFOURCC << "," << (recordColor ? "COLOR" : "NOCOLOR") << ")");
					writers.at(i).open(videoFiles.at(i), fourcc, recordFPS, cv::Size((**inputs.at(i)).cols, (**inputs.at(i)).rows), recordColor);
					if (!writers.at(i).isOpened()) LOG(0, "Could not open writer for '" << videoFiles.at(i));
				}
				writers.at(i).write(**inputs.at(i));
				if (displayMode) cv::imshow("in"+std::to_string(i+1), **inputs.at(i));
			}
			LOG(2, "Writing frame(s) to " << numNodes << " file(s)!");
			break;
		case 'S':
			for (int i=0; i<numNodes; i++) {
				std::string filename = getImageFileName(counterStart, i);
				bool written = cv::imwrite(filename, **inputs.at(i));
				if (!written) LOG(0, "Could not write to file '" << filename << "'!");
				if (displayMode) cv::imshow("in"+std::to_string(i+1), **inputs.at(i));
			}
			LOG(2, "Writing frame(s) to " << numNodes << " file(s)!");
			counterStart += stepSize;
			break;
		case 'D':
			for (int i=0; i<numNodes; i++) {
				if (displayMode) cv::imshow("in"+std::to_string(i+1), **inputs.at(i));
			}
			break;
		default: break;
	}

	for (auto& in: inputs) in->unlockConnection();
	for (auto& out: outputs) out->unlockConnection();

	if (displayMode) cv::waitKey(1);

	return BVS::Status::OK;
}



BVS::Status CaptureCV::debugDisplay()
{
	return BVS::Status::OK;
}



std::string CaptureCV::getImageFileName(int frame, int nodeID)
{
	std::string tmp;
	for (auto& piece: fileNamePieces) {
		if (piece == "FOLDER") {
			std::string f;
			folder.receive(f);
			tmp += f;
		} else if (piece == "FRAME") {
			std::string fr = std::to_string(frame);
			if (fr.size() < (unsigned int)frameNumberPadding)
				fr.insert(fr.begin(), frameNumberPadding-fr.size(), '0');
			tmp += fr;
		} else if (piece == "NODE") tmp += std::to_string(nodeID+1);
		else tmp += piece;
	}

	return tmp;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(CaptureCV)

