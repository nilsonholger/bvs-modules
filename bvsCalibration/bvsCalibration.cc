#include "bvsCalibration.h"
#include "sys/stat.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to the its config.
// However, you should use it to create your data structures etc.
bvsCalibration::bvsCalibration(const std::string id, const BVS::Info& bvs)
	: BVS::Module()
	, id(id)
	, logger(id)
	, bvs(bvs)
	, calibrated(false)
	, detectionRunning(false)
	, numNodes(bvs.config.getValue<int>(id + ".numNodes", 0))
	, numImages(bvs.config.getValue<int>(id + ".numImages", 50))
	, numDetections(0)
	, circleSize(bvs.config.getValue<float>(id + ".circleSize", 2.42f))
	, autoShotMode(bvs.config.getValue<bool>(id + ".autoShotMode", true))
	, autoShotDelay(bvs.config.getValue<int>(id + ".autoShotDelay", 2))
	, directory(bvs.config.getValue<std::string>(id + ".directory", "calibrationData"))
	, saveImages(bvs.config.getValue<bool>(id + ".saveImages", false))
	, loadCalibration(bvs.config.getValue<bool>(id + ".loadCalibration", true))
	, saveCalibration(bvs.config.getValue<bool>(id + ".saveCalibration", true))
	, calibrationFile(bvs.config.getValue<std::string>(id + ".calibrationFile", "calibration.xml"))
	, createRectifiedOutput(bvs.config.getValue<bool>(id + ".createRectifiedOutput", true))
	, addGridOverlay(bvs.config.getValue<bool>(id + ".addGridOverlay", false))
	, nodes()
	, objectPoints()
	, imageSize()
	, boardSize(4, 11)
	, stereo(nodes)
	, detectionThread()
	, detectionMutex()
	, detectionLock(detectionMutex)
	, detectionCond()
	, shotTimer(std::chrono::steady_clock::now())
{
	for (int i=0; i<numNodes; i++)
	{
		nodes.emplace_back(CalibrationNode{
				i
				, BVS::Connector<cv::Mat>(std::string("input") + std::to_string(i), BVS::ConnectorType::INPUT)
				, BVS::Connector<cv::Mat>(std::string("output") + std::to_string(i), BVS::ConnectorType::OUTPUT)
				, cv::Mat()
				, cv::Mat()
				, std::vector<cv::Point2f>()
				, cv::Mat()
				, std::vector<cv::Point2f>()
				, std::vector<std::vector<cv::Point2f>>()
				, cv::Mat::eye(3, 3, CV_64F)
				, cv::Mat()
				, cv::Mat()
				, cv::Mat()
				});
	}

	struct stat *buf = nullptr;
	if (stat(directory.c_str(), buf)) mkdir(directory.c_str(), 0755);

	if (loadCalibration) calibrated = loadCalibrationFrom(directory, calibrationFile);
	
	if (!calibrated)
	{
		detectionThread = std::thread(&bvsCalibration::detectCalibrationPoints, this);
		detectionThread.detach();
	}
}



// This is your module's destructor.
// See the constructor for more info.
bvsCalibration::~bvsCalibration()
{

}



// Put all your work here.
BVS::Status bvsCalibration::execute()
{
	LOG(2, "Execution of " << id << "!");

	for (auto& node: nodes)
	{
		node.input.receive(node.frame);
		if (!calibrated) cv::pyrDown(node.frame, node.scaledFrame, cv::Size(imageSize.width/2, imageSize.height/2));
	}
	if (imageSize == cv::Size()) imageSize = nodes[0].frame.size();

	if (!calibrated && numDetections<numImages) collectCalibrationImages();
	if (!calibrated && numDetections==numImages && !detectionRunning)
	{
		calibrate();
		clearCalibrationData();
		calibrated = true;
	}

	if (calibrated && createRectifiedOutput) rectifyOutput(addGridOverlay);

	static double avgFPS = 15;
	// apply exponential smoothing with alpha = 0.05
	double duration = bvs.lastRoundDuration.count();
	duration = duration > 1000 ? 1000 : duration;
	avgFPS = (1000/duration + 19 * avgFPS)/20;
	std::string fps = std::to_string(avgFPS);
	fps.resize(fps.length()-5);

	for (auto& node: nodes)
	{
		if (calibrated)
		{
			cv::putText(*(node.output),fps, cv::Point(10, 30),
					CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(0, 0, 255), 2);
			cv::imshow(std::to_string(node.id), *(node.output));
			cv::moveWindow(std::to_string(node.id), node.id*node.output->cols, 0);
		}
		else
		{
			cv::putText(node.scaledFrame,fps, cv::Point(10, 30),
					CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(0, 0, 255), 2);
			cv::putText(node.scaledFrame, std::to_string(numDetections) + "/" + std::to_string(numImages),
					cv::Point(10, 70), CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(0, 255, 0), 2);

			if (!node.scaledFrame.empty()) cv::imshow(std::to_string(node.id), node.scaledFrame);
			cv::moveWindow(std::to_string(node.id), node.id*node.scaledFrame.cols, 0);
		}

	}

	char c = cv::waitKey(1);
	if (c==27) exit(0);
	if (!autoShotMode && c==' ') notifyDetectionThread();

	return BVS::Status::OK;
}



// UNUSED
BVS::Status bvsCalibration::debugDisplay()
{
	return BVS::Status::OK;
}



// This function is called by the framework upon creating a module instance of
// this class. It creates the module and registers it within the framework.
// DO NOT CHANGE OR DELETE
extern "C" {
	int bvsRegisterModule(std::string id, const BVS::Info& bvs)
	{
		registerModule(id, new bvsCalibration(id, bvs));

		return 0;
	}
}



bool bvsCalibration::loadCalibrationFrom(const std::string& directory, const std::string& file)
{
	switch (numNodes)
	{
		case 1:
			// single camera
			return false;
			break;
		case 2:
			// stereo
			return stereo.loadFromFile(directory, file);
			break;
	}
	return false;
}



bool bvsCalibration::saveCalibrationTo(const std::string& directory, const std::string& file)
{
	switch (numNodes)
	{
		case 1:
			// single camera
			return false;
			break;
		case 2:
			// stereo
			return stereo.saveToFile(directory, file);
			break;
	}
	return false;
}



void bvsCalibration::calibrate()
{
	switch (numNodes)
	{
		case 1:
			// single camera
			break;
		case 2:
			// stereo
			stereo.calibrate(numDetections, imageSize, boardSize, circleSize);
			if (saveCalibration) saveCalibrationTo(directory, calibrationFile);
			break;
	}
}



void bvsCalibration::rectifyOutput(bool addGridOverlay)
{
	switch (numNodes)
	{
		case 1:
			// single camera
			break;
		case 2:
			// stereo
			stereo.rectify(addGridOverlay);
			break;
	}
}



void bvsCalibration::collectCalibrationImages()
{
	bool foundPattern = false;
	int numPositives = 0;

	for (auto& node: nodes)
	{
		foundPattern = cv::findCirclesGrid(node.scaledFrame, boardSize,
				node.framePoints, cv::CALIB_CB_ASYMMETRIC_GRID);
		cv::drawChessboardCorners(node.scaledFrame, boardSize,
				cv::Mat(node.framePoints), foundPattern);

		if (!foundPattern) break;
		if(foundPattern) numPositives++;
		if (numPositives==numNodes)
		{
			if (!autoShotMode) return;
			notifyDetectionThread();
		}
	}
}



void bvsCalibration::notifyDetectionThread()
{
	if (detectionRunning) return;

	if (autoShotMode && std::chrono::duration_cast<std::chrono::seconds>
			(std::chrono::steady_clock::now() - shotTimer).count() < autoShotDelay)
		return;
	
	shotTimer = std::chrono::steady_clock::now();
	numDetections++;
	for (auto& node: nodes)
		node.sample = node.frame.clone();
	detectionRunning = true;
	detectionCond.notify_one();
}



void bvsCalibration::detectCalibrationPoints()
{
	bool foundPattern;
	int numPositives;

	for (auto& node: nodes)
		node.pointStore.resize(numImages);

	while (numDetections<numImages)
	{
		detectionCond.wait(detectionLock, [&](){ return detectionRunning; });
		numPositives = 0;
		for (auto& node: nodes)
		{
			foundPattern = cv::findCirclesGrid(node.sample, boardSize, node.points, cv::CALIB_CB_ASYMMETRIC_GRID);
			if (!foundPattern)
			{
				numDetections--;
				break;
			}
			if(foundPattern) numPositives++;
			if (numPositives==numNodes)
			{
				for (auto& node: nodes)
					node.pointStore.at(numDetections-1) = node.points;
				if (saveImages)
					cv::imwrite(std::string(directory + "/img") + std::to_string(numDetections)
							+ "-" + std::to_string(node.id) + ".pbm", node.sample);
			}
		}
		detectionRunning = false;
	}
}



void bvsCalibration::clearCalibrationData()
{
	for (auto& node: nodes)
	{
		node.scaledFrame.release();
		node.framePoints.clear();
		node.framePoints.shrink_to_fit();
		node.sample.release();
		node.points.clear();
		node.points.shrink_to_fit();
		node.pointStore.clear();
		node.pointStore.shrink_to_fit();
	}
}
