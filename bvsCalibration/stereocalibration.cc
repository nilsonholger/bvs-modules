#include "stereocalibration.h"
#include "opencv2/opencv.hpp"



StereoCalibration::StereoCalibration(CalNodeVec& nodes)
	: logger("StereoCalib")
	, nodes(nodes)
	, imageSize()
	, rms(0)
	, averageError(0)
	, objectPoints()
	, stereoRotation()
	, stereoTranslation()
	, stereoEssential()
	, stereoFundamental()
	, disparityToDepthMapping()
	, rectifyMap{{cv::Mat(), cv::Mat()}, {cv::Mat(), cv::Mat()}}
{ }



bool StereoCalibration::loadFromFile(const std::string& path, const std::string& file)
{
	cv::FileStorage fs(path + "/" + file, cv::FileStorage::READ);

	if (!fs.isOpened()) return false;

	fs["imageWidth"] >> imageSize.width;
	fs["imageHeigth"] >> imageSize.height;
	fs["rms"] >> rms;
	fs["avgerageError"] >> averageError;
	fs["stereoRotation"] >> stereoRotation;
	fs["stereoTranslation"] >> stereoTranslation;
	fs["stereoEssential"] >> stereoEssential;
	fs["stereoFundamental"] >> stereoFundamental;
	fs["disparityToDepthMapping"] >> disparityToDepthMapping;

	for (auto& node: nodes)
	{
		std::string id = std::to_string(node.id);
		fs["cameraMatrix"+id] >> node.cameraMatrix;
		fs["distCoeffs"+id] >> node.distCoeffs;
		fs["rectificationMatrix"+id] >> node.rectificationMatrix;
		fs["projectionMatrix"+id] >> node.projectionMatrix;
	}

	return true;
}



bool StereoCalibration::saveToFile(const std::string& path, const std::string& file)
{
	cv::FileStorage fs(path + "/" + file , cv::FileStorage::WRITE);

	if (!fs.isOpened()) return false;

	fs << "imageWidth" << imageSize.width;
	fs << "imageHeigth" << imageSize.height;
	fs << "rms" << rms;
	fs << "avgerageError" << averageError;
	fs << "stereoRotation" << stereoRotation;
	fs << "stereoTranslation" << stereoTranslation;
	fs << "stereoEssential" << stereoEssential;
	fs << "stereoFundamental" << stereoFundamental;
	fs << "disparityToDepthMapping" << disparityToDepthMapping;

	for (auto& node: nodes)
	{
		std::string id = std::to_string(node.id);
		fs << "cameraMatrix"+id << node.cameraMatrix;
		fs << "distCoeffs"+id << node.distCoeffs;
		fs << "rectificationMatrix"+id << node.rectificationMatrix;
		fs << "projectionMatrix"+id << node.projectionMatrix;
	}

	return true;
}



void StereoCalibration::calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, float circleSize)
{
	this->imageSize = imageSize;

	// generate a priori object points
	objectPoints.resize(numImages);
	for (int i=0; i<numImages; i++)
		for (int j=0; j<boardSize.height; j++)
			for (int k=0; k<boardSize.width; k++)
				objectPoints[i].push_back(cv::Point3f(j*circleSize, k*circleSize, 0));

	LOG(2, "calibrating individual cameras intrinsics!");
	std::vector<std::thread> threads;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	for (auto& node: nodes)
	{
		threads.push_back(std::thread([&]{cv::calibrateCamera(
						objectPoints,
						node.pointStore,
						imageSize,
						node.cameraMatrix,
						node.distCoeffs,
						rvecs,
						tvecs,
						CV_CALIB_RATIONAL_MODEL,
						cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
						);}));
	}
	for (auto& t: threads) t.join();

	LOG(2, "calibrating stereo!");
	rms = cv::stereoCalibrate(
			objectPoints,
			nodes[0].pointStore,
			nodes[1].pointStore,
			nodes[0].cameraMatrix,
			nodes[0].distCoeffs,
			nodes[1].cameraMatrix,
			nodes[1].distCoeffs,
			imageSize,
			stereoRotation,
			stereoTranslation,
			stereoEssential,
			stereoFundamental,
			cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			CV_CALIB_USE_INTRINSIC_GUESS +
			CV_CALIB_FIX_ASPECT_RATIO +
			CV_CALIB_SAME_FOCAL_LENGTH +
			CV_CALIB_ZERO_TANGENT_DIST +
			CV_CALIB_RATIONAL_MODEL +
			CV_CALIB_FIX_K3 +
			CV_CALIB_FIX_K4 +
			CV_CALIB_FIX_K5);
	LOG(1, "reprojection error: " << rms);



	int npoints = 0;
	std::vector<cv::Vec3f> lines[2];

	for(int i = 0; i < numImages; i++ )
	{
		int npt = nodes[0].pointStore[i].size();
		cv::Mat imgpt[2];
		for(int k = 0; k < 2; k++ )
		{
			imgpt[k] = cv::Mat(nodes[k].pointStore[i]);
			undistortPoints(imgpt[k], imgpt[k], nodes[k].cameraMatrix, nodes[k].distCoeffs, cv::Mat(), nodes[k].cameraMatrix);
			computeCorrespondEpilines(imgpt[k], k+1, stereoFundamental, lines[k]);
		}
		for(int j = 0; j < npt; j++ )
		{
			double errIJ =
				fabs(nodes[0].pointStore[i][j].x*lines[1][j][0] +
				nodes[0].pointStore[i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(nodes[0].pointStore[i][j].x*lines[0][j][0] +
				nodes[1].pointStore[i][j].y*lines[0][j][1] + lines[0][j][2]);
			averageError += errIJ;
		}
		npoints += npt;
	}
	averageError /= npoints;
	LOG(1, "average calibration error: " << averageError);



	cv::Rect validRoi[2];

	cv::stereoRectify(
			nodes[0].cameraMatrix, nodes[0].distCoeffs,
			nodes[1].cameraMatrix, nodes[1].distCoeffs,
			imageSize,
			stereoRotation,
			stereoTranslation,
			nodes[0].rectificationMatrix, nodes[1].rectificationMatrix,
			nodes[0].projectionMatrix, nodes[1].projectionMatrix,
			disparityToDepthMapping,
			CV_CALIB_ZERO_DISPARITY, 1, imageSize,
			&validRoi[0], &validRoi[1]);

	std::vector<cv::Point2f> allimgpt[2];
	for( int k = 0; k < 2; k++ )
	{
		for( int i = 0; i < numImages; i++ )
			std::copy(nodes[k].pointStore[i].begin(), nodes[k].pointStore[i].end(), back_inserter(allimgpt[k]));
	}
	stereoFundamental = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), CV_FM_8POINT, 0, 0);
	cv::Mat H1, H2;
	cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), stereoFundamental, imageSize, H1, H2, 3);

	nodes[0].rectificationMatrix = nodes[0].cameraMatrix.inv()*H1*nodes[0].cameraMatrix;
	nodes[1].rectificationMatrix = nodes[1].cameraMatrix.inv()*H2*nodes[1].cameraMatrix;
	nodes[0].projectionMatrix = nodes[0].cameraMatrix;
	nodes[1].projectionMatrix = nodes[1].cameraMatrix;
}



void StereoCalibration::rectify(bool addGridOverlay)
{
	static bool initRectifyMap = true;

	if (initRectifyMap)
	{
		cv::initUndistortRectifyMap(nodes[0].cameraMatrix, nodes[0].distCoeffs,
				nodes[0].rectificationMatrix, nodes[0].projectionMatrix, imageSize,
				CV_16SC2, rectifyMap[0][0], rectifyMap[0][1]);
		cv::initUndistortRectifyMap(nodes[1].cameraMatrix, nodes[1].distCoeffs,
				nodes[1].rectificationMatrix, nodes[1].projectionMatrix, imageSize,
				CV_16SC2, rectifyMap[1][0], rectifyMap[1][1]);
		initRectifyMap = false;
	}

	cv::remap(nodes[0].frame, *nodes[0].output, rectifyMap[0][0], rectifyMap[0][1], CV_INTER_LINEAR);
	cv::remap(nodes[1].frame, *nodes[1].output, rectifyMap[1][0], rectifyMap[1][1], CV_INTER_LINEAR);

	if (addGridOverlay)
	{
		for (auto& node: nodes)
		{
			for(int i = 0; i < node.output->rows; i += node.output->rows/10)
				cv::line(*node.output, cv::Point(0, i), cv::Point(node.output->cols, i), cv::Scalar(0, 255, 0), 1, 8);
			for(int i = 0; i < node.output->cols; i += node.output->cols/10)
				cv::line(*node.output, cv::Point(i, 0), cv::Point(i, node.output->rows), cv::Scalar(0, 255, 0), 1, 8);
		}
	}
}
