#include "stereocalibration.h"
#include "opencv2/opencv.hpp"
#include<iostream>



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
{
	if (nodes.size()!=2)
	{
		std::cerr << "Size of input nodes vector is not 2!" << std::endl;
		exit(1);
	}
}



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
		std::string id = std::to_string(node->id);
		fs["cameraMatrix"+id] >> node->cameraMatrix;
		fs["distCoeffs"+id] >> node->distCoeffs;
		fs["rectificationMatrix"+id] >> node->rectificationMatrix;
		fs["projectionMatrix"+id] >> node->projectionMatrix;
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
		std::string id = std::to_string(node->id);
		fs << "cameraMatrix"+id << node->cameraMatrix;
		fs << "distCoeffs"+id << node->distCoeffs;
		fs << "rectificationMatrix"+id << node->rectificationMatrix;
		fs << "projectionMatrix"+id << node->projectionMatrix;
	}

	return true;
}



void StereoCalibration::calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, float blobSize)
{
	this->imageSize = imageSize;

	// generate a priori object points of detection pattern
	objectPoints.resize(numImages);
	for (int i=0; i<numImages; i++)
		for (int j=0; j<boardSize.height; j++)
			for (int k=0; k<boardSize.width; k++)
				objectPoints.at(i).push_back(cv::Point3f(j*blobSize, k*blobSize, 0));

	LOG(2, "calibrating individual cameras intrinsics!");
	std::vector<std::thread> threads;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	for (auto& node: nodes)
	{
		threads.push_back(std::thread([&]{cv::calibrateCamera(
						objectPoints,
						node->pointStore,
						imageSize,
						node->cameraMatrix,
						node->distCoeffs,
						rvecs,
						tvecs,
						CV_CALIB_FIX_PRINCIPAL_POINT +
						CV_CALIB_FIX_ASPECT_RATIO +
						CV_CALIB_ZERO_TANGENT_DIST +
						CV_CALIB_SAME_FOCAL_LENGTH +
						CV_CALIB_RATIONAL_MODEL +
						CV_CALIB_FIX_K3 +
						CV_CALIB_FIX_K4 +
						CV_CALIB_FIX_K5,
						cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
						);}));
	}
	for (auto& t: threads) t.join();


	LOG(2, "calibrating stereo!");
	rms = cv::stereoCalibrate(
			objectPoints, nodes.at(0)->pointStore, nodes.at(1)->pointStore,
			nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs, nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
			imageSize, stereoRotation, stereoTranslation, stereoEssential, stereoFundamental,
			cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			CV_CALIB_USE_INTRINSIC_GUESS +
			CV_CALIB_FIX_PRINCIPAL_POINT +
			CV_CALIB_FIX_ASPECT_RATIO +
			CV_CALIB_SAME_FOCAL_LENGTH +
			CV_CALIB_RATIONAL_MODEL +
			CV_CALIB_FIX_K3 +
			CV_CALIB_FIX_K4 +
			CV_CALIB_FIX_K5);
	LOG(1, "reprojection error: " << rms);



	LOG(2, "calculating average calibration error!");
	int points = 0;
	int sumPoints = 0;
	std::vector<cv::Vec3f> lines[2];

	for(int i = 0; i < numImages; i++ )
	{
		points = nodes[0]->pointStore.at(i).size();
		cv::Mat imgpt[2];
		for(int k = 0; k < 2; k++ )
		{
			imgpt[k] = cv::Mat(nodes[k]->pointStore.at(i));
			cv::undistortPoints(imgpt[k], imgpt[k], nodes.at(k)->cameraMatrix, nodes.at(k)->distCoeffs, cv::Mat(), nodes.at(k)->cameraMatrix);
			cv::computeCorrespondEpilines(imgpt[k], k+1, stereoFundamental, lines[k]);
		}
		for(int j = 0; j < points; j++ )
		{
			double errIJ =
				fabs(nodes.at(0)->pointStore[i][j].x*lines[1][j][0] +
				nodes.at(0)->pointStore[i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(nodes.at(0)->pointStore[i][j].x*lines[0][j][0] +
				nodes.at(0)->pointStore[i][j].y*lines[0][j][1] + lines[0][j][2]);
			averageError += errIJ;
		}
		sumPoints += points;
	}
	averageError /= sumPoints;
	LOG(1, "average calibration error: " << averageError);



	LOG(2, "calculating stereo rectification!");
	cv::stereoRectify(
			nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs,
			nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
			imageSize, stereoRotation, stereoTranslation,
			nodes.at(0)->rectificationMatrix, nodes.at(1)->rectificationMatrix,
			nodes.at(0)->projectionMatrix, nodes.at(1)->projectionMatrix,
			disparityToDepthMapping, CV_CALIB_ZERO_DISPARITY, 1, imageSize,
			&nodes.at(0)->validRegionOfInterest, &nodes.at(1)->validRegionOfInterest);



	LOG(2, "using fundamental matrix to calculate rectification and projection matrices!");
	std::vector<cv::Point2f> allimgpt[2];
	for( int k = 0; k < 2; k++ )
	{
		for( int i = 0; i < numImages; i++ )
			std::copy(nodes.at(k)->pointStore.at(i).begin(), nodes.at(k)->pointStore.at(i).end(), back_inserter(allimgpt[k]));
	}
	//TODO test RANSAC and others, or use fundamental from stereoCalibrate, compare quality
	stereoFundamental = cv::findFundamentalMat(cv::Mat(allimgpt[0]),
			cv::Mat(allimgpt[1]), CV_FM_8POINT, 0, 0);
	cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]),
			stereoFundamental, imageSize, nodes.at(0)->homographyMatrix, nodes.at(1)->homographyMatrix, 3);

	nodes.at(0)->rectificationMatrix = nodes.at(0)->cameraMatrix.inv()*nodes.at(0)->homographyMatrix*nodes.at(0)->cameraMatrix;
	nodes.at(1)->rectificationMatrix = nodes.at(1)->cameraMatrix.inv()*nodes.at(1)->homographyMatrix*nodes.at(1)->cameraMatrix;
	nodes.at(0)->projectionMatrix = nodes.at(0)->cameraMatrix;
	nodes.at(1)->projectionMatrix = nodes.at(1)->cameraMatrix;
	
	LOG(2, "calibration done!");
}



void StereoCalibration::rectify(bool addGridOverlay)
{
	static bool initRectifyMap = true;

	if (initRectifyMap)
	{
		cv::initUndistortRectifyMap(nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs,
				nodes.at(0)->rectificationMatrix, nodes.at(0)->projectionMatrix, imageSize,
				CV_16SC2, rectifyMap[0][0], rectifyMap[0][1]);
		cv::initUndistortRectifyMap(nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
				nodes.at(1)->rectificationMatrix, nodes.at(1)->projectionMatrix, imageSize,
				CV_16SC2, rectifyMap[1][0], rectifyMap[1][1]);
		initRectifyMap = false;
	}

	cv::remap(nodes.at(0)->frame, *nodes.at(0)->output, rectifyMap[0][0], rectifyMap[0][1], CV_INTER_LINEAR);
	cv::remap(nodes.at(1)->frame, *nodes.at(1)->output, rectifyMap[1][0], rectifyMap[1][1], CV_INTER_LINEAR);

	if (addGridOverlay)
	{
		for (auto& node: nodes)
		{
			for(int i = 0; i < node->output->rows; i += node->output->rows/10)
				cv::line(*node->output, cv::Point(0, i), cv::Point(node->output->cols, i), cv::Scalar(0, 255, 0), 1, 8);
			for(int i = 0; i < node->output->cols; i += node->output->cols/10)
				cv::line(*node->output, cv::Point(i, 0), cv::Point(i, node->output->rows), cv::Scalar(0, 255, 0), 1, 8);
		}
	}
}

