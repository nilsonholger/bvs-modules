#include "CalibrationCV.h"
#include "sys/stat.h"



CalibrationCV::CalibrationCV(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module()
	, info(info)
	, logger(info.id)
	, bvs(bvs)
	, numNodes(bvs.config.getValue<int>(info.conf + ".numNodes", 0))
	, directory(bvs.config.getValue<std::string>(info.conf + ".directory", "calibrationData"))
	, useCalibrationFile(bvs.config.getValue<std::string>(info.conf + ".useCalibrationFile", "calibration.xml"))
	, saveImages(bvs.config.getValue<bool>(info.conf + ".saveImages", false))
	, useSavedImages(bvs.config.getValue<bool>(info.conf + ".useSavedImages", false))
	, imageDirectory(bvs.config.getValue<std::string>(info.conf + ".imageDirectory", "calibrationImages"))
	, autoShotDelay(bvs.config.getValue<int>(info.conf + ".autoShotDelay", 1))
	, numImages(bvs.config.getValue<int>(info.conf + ".numImages", 45))
	, fisheye(bvs.config.getValue<bool>(info.conf + ".fisheye", false))
	, pattern(bvs.config.getValue<std::string>(info.conf + ".pattern", "ASYMMETRIC"))
	, gridBlobSize(bvs.config.getValue<float>(info.conf + ".gridBlobSize", 1.0f))
	, gridX(bvs.config.getValue<unsigned int>(info.conf + ".gridX", 4))
	, gridY(bvs.config.getValue<unsigned int>(info.conf + ".gridY", 11))
	, rectifyOutput(bvs.config.getValue<bool>(info.conf + ".rectifyOutput", true))
	, addGridOverlay(bvs.config.getValue<bool>(info.conf + ".addGridOverlay", false))
	, rectifyCalImages(bvs.config.getValue<bool>(info.conf + ".rectifyCalImages", false))
	, rectifiedDirectory(bvs.config.getValue<std::string>(info.conf + ".outputDirectory", "rectifiedImages"))
	, useCalibrationGuide(bvs.config.getValue<bool>(info.conf + ".useCalibrationGuide", false))
	, sectorDetections(bvs.config.getValue<int>(info.conf + ".sectorDetections", 5))
	, calibrated(false)
	, detectionRunning(false)
	, numDetections(0)
	, rectifyCounter(1)
	, objectPoints()
	, imageSize()
	, boardSize(gridX, gridY)
	, flags{pattern.at(0)=='A' ? cv::CALIB_CB_ASYMMETRIC_GRID : cv::CALIB_CB_SYMMETRIC_GRID}
	, detectionThread()
	, detectionMutex()
	, detectionLock(detectionMutex)
	, detectionCond()
	, shotTimer(std::chrono::high_resolution_clock::now())
	, reflectX()
	, reflectY()
	, nodes()
	, stereo(nodes, fisheye)
	, guide(numImages, numDetections, sectorDetections)
{
	if (numNodes==0) LOG(0, "numNodes == 0!!!");

	for (int i=0; i<numNodes; i++) {
		nodes.emplace_back(new CalibrationNode(
				i+1,
				BVS::Connector<cv::Mat>(std::string("in") + std::to_string(i+1), BVS::ConnectorType::INPUT),
				BVS::Connector<cv::Mat>(std::string("out") + std::to_string(i+1), BVS::ConnectorType::OUTPUT),
				cv::Mat(), cv::Mat(), std::vector<cv::Point2f>(), cv::Mat(),
				std::vector<cv::Point2f>(), std::vector<std::vector<cv::Point2f>>(),
				cv::Mat::eye(3, 3, CV_64F), cv::Mat(), cv::Mat(), cv::Mat(), cv::Rect()));
	}

	struct stat *buf = nullptr;
	if (stat(directory.c_str(), buf)) mkdir(directory.c_str(), 0755);
	if (saveImages) {
		std::string tmp = directory + "/" + imageDirectory;
		if (stat(tmp.c_str(), buf)) mkdir(tmp.c_str(), 0755);
	}
	if (rectifyCalImages) {
		std::string tmp = directory + "/" + rectifiedDirectory;
		if (stat(tmp.c_str(), buf)) mkdir(tmp.c_str(), 0755);
	}

	if (!useCalibrationFile.empty()) calibrated = loadCalibrationFrom(directory, useCalibrationFile);
	if (!calibrated && !useSavedImages) detectionThread = std::thread(&CalibrationCV::detectCalibrationPoints, this);
	if (!(calibrated || useSavedImages))
		for (auto& node: nodes) cv::namedWindow(info.id+"_"+std::to_string(node->id));
}



CalibrationCV::~CalibrationCV()
{
	if (detectionThread.joinable()) {
		numDetections = numImages;
		notifyDetectionThread();
		detectionThread.join();
	}

	for (auto& it: nodes) delete it;
}



BVS::Status CalibrationCV::execute()
{
	if (useSavedImages) {
		if (!calibrated) calibrateUsingSavedImages();
		if (calibrated && rectifyCalImages) rectifyCalibrationImages();
		return BVS::Status::SHUTDOWN;
	} else {
		for (auto& node: nodes) {
			if(!node->input.receive(node->frame)) return BVS::Status::NOINPUT;
			if (node->frame.empty()) return BVS::Status::NOINPUT;
		}
		if (imageSize == cv::Size()) {
			imageSize = nodes[0]->frame.size();
			generateReflectionMap();
		}
		if (!calibrated) {
			for (auto& node: nodes) {
				cv::remap(node->frame, *node->output, reflectX, reflectY, 0);
				if(node->output->type()==CV_8UC1) cv::cvtColor(*node->output, *node->output, cv::COLOR_GRAY2BGR);
			}
			if (useCalibrationGuide) guide.addTargetOverlay(*nodes[0]->output);
			if (numDetections<numImages) collectCalibrationImages();
			if (numDetections==numImages && !detectionRunning) calibrate();
			for (auto& node: nodes) {
				cv::putText(*node->output, bvs.getFPS(), cv::Point(10, 30),
						cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(0, 0, 255), 2);
				cv::putText(*node->output, std::to_string(numDetections) + "/" + std::to_string(numImages),
						cv::Point(100, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0f, cv::Scalar(0, 255, 0), 2, 8);
				cv::imshow(info.id+"_"+std::to_string(node->id), *node->output);
				if (calibrated) cv::destroyWindow(std::to_string(node->id));
			}
			char c = cv::waitKey(1);
			if (c==27) return BVS::Status::SHUTDOWN;
			if (autoShotDelay==0 && c==' ') notifyDetectionThread();
		}
		else if (rectifyOutput) rectifyOutputNodes();
		else for (auto& node: nodes) *node->output = node->frame;
	}

	return BVS::Status::OK;
}



// UNUSED
BVS::Status CalibrationCV::debugDisplay()
{
	return BVS::Status::OK;
}



bool CalibrationCV::loadCalibrationFrom(const std::string& directory, const std::string& file)
{
	switch (numNodes) {
		case 1:
			// single camera
			return false;
			break;
		case 2:
			return stereo.loadFromFile(directory, file);
			break;
	}
	return false;
}



bool CalibrationCV::saveCalibrationTo(const std::string& directory, const std::string& file)
{
	switch (numNodes) {
		case 1:
			// single camera
			return false;
			break;
		case 2:
			return stereo.saveToFile(directory, file);
			break;
	}
	return false;
}



void CalibrationCV::generateReflectionMap()
{
	cv::Mat mapX;
	cv::Mat mapY;
	mapX.create(imageSize, CV_32FC1);
	mapY.create(imageSize, CV_32FC1);
	for (int i=0; i<imageSize.height; i++)
		for (int j=0; j<imageSize.width; j++) {
			mapX.at<float>(i,j) = imageSize.width-j;
			mapY.at<float>(i,j) = i;
		}
	cv::convertMaps(mapX, mapY, reflectX, reflectY, CV_16SC2);
}



void CalibrationCV::calibrate()
{
	switch (numNodes) {
		case 1:
			// single camera
			break;
		case 2:
			stereo.calibrate(numDetections, imageSize, boardSize, pattern, gridBlobSize);
			if (!useCalibrationFile.empty()) saveCalibrationTo(directory, useCalibrationFile);
			break;
	}

	clearCalibrationData();
	calibrated = true;
}



void CalibrationCV::rectifyOutputNodes()
{
	switch (numNodes) {
		case 1:
			// single camera
			break;
		case 2:
			stereo.rectify(imageSize, addGridOverlay);
			break;
	}
}



void CalibrationCV::collectCalibrationImages()
{
	bool foundPattern = false;
	int numPositives = 0;

	for (auto& node: nodes) {
		/** @todo paralellize with threads to decrease latency? */
		cv::pyrDown(node->frame, node->scaledFrame, cv::Size(imageSize.width/2, imageSize.height/2));
		foundPattern = cv::findCirclesGrid(node->scaledFrame, boardSize,
				node->framePoints, flags);
		for (auto& point: node->framePoints) {
			point.x = node->frame.cols - point.x * 2;
			point.y = point.y * 2;
		}
		cv::drawChessboardCorners(*node->output, boardSize,
				cv::Mat(node->framePoints), foundPattern);

		if (!foundPattern) {
			cv::putText(*nodes[0]->output, "Pattern NOT FOUND!",
					cv::Point(10, imageSize.height-10), cv::FONT_HERSHEY_DUPLEX,
					1.0f, cv::Scalar(0, 0, 255));
			break;
		}
		else numPositives++;
		if (numPositives==numNodes) {
			if (autoShotDelay==0) return;
			if (useCalibrationGuide)
				if (!guide.checkDetectionQuality(*nodes[0]->output, nodes[0]->framePoints))
					return;
			notifyDetectionThread();
		}
	}
}



void CalibrationCV::notifyDetectionThread()
{
	if (detectionRunning) return;

	if (autoShotDelay!=0 && std::chrono::duration_cast<std::chrono::seconds>
			(std::chrono::high_resolution_clock::now() - shotTimer).count() < autoShotDelay && !useSavedImages)
		return;

	shotTimer = std::chrono::high_resolution_clock::now();
	numDetections++;
	for (auto& node: nodes)
		node->sample = node->frame.clone();
	detectionRunning = true;
	detectionCond.notify_one();
}



void CalibrationCV::detectCalibrationPoints()
{
	BVS::nameThisThread("detectCalib");
	bool foundPattern;
	int numPositives;

	for (auto& node: nodes)
		node->pointStore.resize(numImages);

	while (numDetections<numImages) {
		detectionCond.wait(detectionLock, [&](){ return detectionRunning; });
		numPositives = 0;
		for (auto& node: nodes) {
			foundPattern = cv::findCirclesGrid(node->sample, boardSize, node->points, flags);
			if (!foundPattern) {
				numDetections--;
				break;
			} else numPositives++;

			if (numPositives==numNodes) {
				for (auto& node: nodes) {
					node->pointStore.at(numDetections-1) = node->points;
					if (saveImages)
						cv::imwrite(directory + "/" + imageDirectory + "/img"
								+ std::to_string(numDetections) + "-" + std::to_string(node->id)
								+ ".pbm", node->sample);
				}
			}
		}
		detectionRunning = false;
	}
}



void CalibrationCV::clearCalibrationData()
{
	for (auto& node: nodes) {
		node->scaledFrame.release();
		node->framePoints.clear();
		node->framePoints.shrink_to_fit();
		node->sample.release();
		node->points.clear();
		node->points.shrink_to_fit();
		node->pointStore.clear();
		node->pointStore.shrink_to_fit();
	}
}



void CalibrationCV::calibrateUsingSavedImages()
{
	for (auto& node: nodes)
		node->pointStore.resize(numImages);

	for (; numDetections<numImages; numDetections++) {
		LOG(2, "processing image: " << numDetections+1 << "/" << numImages);
		for (auto& node: nodes) {
			std::string file = directory + "/" + imageDirectory + "/img" + std::to_string(numDetections+1) + "-" + std::to_string(node->id) + ".pbm";
			node->frame = cv::imread(file);
			if (node->frame.empty()) LOG(0, "image not found: " << file);
			if (imageSize == cv::Size()) imageSize = nodes[0]->frame.size();
			if (!cv::findCirclesGrid(node->frame, boardSize, node->points, flags))
				LOG(0, "could not find grid in image: " << file);
			node->pointStore.at(numDetections) = node->points;
		}
	}

	calibrate();
}



bool CalibrationCV::rectifyCalibrationImages()
{
	for (; rectifyCounter<=numImages; rectifyCounter++)
	{
		LOG(2, "rectifying image " << rectifyCounter << "/" << numImages);

		for (auto& node: nodes) {
			std::string file = directory + "/" + imageDirectory + "/img" + std::to_string(rectifyCounter) + "-" + std::to_string(node->id) + ".pbm";
			node->frame = cv::imread(file);
			if (node->frame.empty()) {
				LOG(0, "image not found: " << file);
				return false;
			}
			if (imageSize == cv::Size()) imageSize = nodes[0]->frame.size();
		}
		rectifyOutputNodes();
		for (auto& node: nodes) cv::imwrite(directory + "/" + rectifiedDirectory + "/rect" + std::to_string(rectifyCounter) + "-" + std::to_string(node->id) + ".jpg", *node->output);
	}
	return true;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(CalibrationCV)

