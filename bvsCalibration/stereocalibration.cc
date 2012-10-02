#include "stereocalibration.h"
#include "bvs/archutils.h"
#include "opencv2/opencv.hpp"
#include <iostream>



StereoCalibration::StereoCalibration(CalNodeVec& nodes)
	: logger("StereoCalib"),
	nodes(nodes),
	imageSize(),
	rms(0),
	objectPoints(),
	stereoRotation(),
	stereoTranslation(),
	stereoEssential(),
	stereoFundamental(),
	disparityToDepthMapping(),
	rectifyMap{{cv::Mat(), cv::Mat()}, {cv::Mat(), cv::Mat()}}
{ }



bool StereoCalibration::loadFromFile(const std::string& path, const std::string& file)
{
	cv::FileStorage fs(path + "/" + file, cv::FileStorage::READ);

	if (!fs.isOpened()) return false;

	fs["rms"] >> rms;
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
		fs["validRegionOfInterest"+id+"x"] >> node->validRegionOfInterest.x;
		fs["validRegionOfInterest"+id+"y"] >> node->validRegionOfInterest.y;
		fs["validRegionOfInterest"+id+"width"] >> node->validRegionOfInterest.width;
		fs["validRegionOfInterest"+id+"height"] >> node->validRegionOfInterest.height;
	}

	return true;
}



bool StereoCalibration::saveToFile(const std::string& path, const std::string& file)
{
	cv::FileStorage fs(path + "/" + file , cv::FileStorage::WRITE);

	if (!fs.isOpened()) return false;

	fs << "rms" << rms;
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
		fs << "validRegionOfInterest"+id+"x" << node->validRegionOfInterest.x;
		fs << "validRegionOfInterest"+id+"y" << node->validRegionOfInterest.y;
		fs << "validRegionOfInterest"+id+"width" << node->validRegionOfInterest.width;
		fs << "validRegionOfInterest"+id+"height" << node->validRegionOfInterest.height;
	}

	return true;
}



void StereoCalibration::calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, float blobSize)
{
	if (nodes.size()!=2)
	{
		std::cerr << "Size of input nodes vector is not 2!" << std::endl;
		exit(1);
	}
	this->imageSize = imageSize;

	// generate a priori object points of detection pattern
	objectPoints.resize(numImages);
	for (int i=0; i<numImages; i++)
		for (int j=0; j<boardSize.height; j++)
			for (int k=0; k<boardSize.width; k++)
				// below works for CHESSBOARDS or SYMMETRIC CIRCLE patterns only
				//objectPoints.at(i).push_back(cv::Point3f(j*blobSize, k*blobSize, 0));
				// below is for ASYMMETRIC CIRCLE patterns
				objectPoints.at(i).push_back(cv::Point3f(double((2*k+j%2)*blobSize/2.), double(j*blobSize/2.), 0.));

	LOG(2, "calibrating individual cameras intrinsics!");
	std::vector<std::thread> threads;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	for (auto& node: nodes)
	{
		threads.push_back(std::thread([&]{
					BVS::nameThisThread("calib.intrinsic");
					double calError = cv::calibrateCamera(objectPoints, node->pointStore,
						imageSize, node->cameraMatrix, node->distCoeffs, rvecs, tvecs,
						CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO +
						CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH +
						CV_CALIB_RATIONAL_MODEL +
						CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5,
						cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
						);
					(void) calError;
					LOG(1, "reprojection error for node " << node->id << ": " << calError);
					}));
	}
	for (auto& t: threads) t.join();

	LOG(2, "calibrating stereo!");
	rms = cv::stereoCalibrate(
			objectPoints, nodes.at(0)->pointStore, nodes.at(1)->pointStore,
			nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs, nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
			imageSize, stereoRotation, stereoTranslation, stereoEssential, stereoFundamental,
			cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			CV_CALIB_USE_INTRINSIC_GUESS + CV_CALIB_FIX_PRINCIPAL_POINT + CV_CALIB_FIX_ASPECT_RATIO +
			CV_CALIB_SAME_FOCAL_LENGTH + CV_CALIB_RATIONAL_MODEL + CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	LOG(1, "stereo reprojection error: " << rms);

	LOG(2, "calculating stereo rectification!");
	cv::stereoRectify(
			nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs,
			nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
			imageSize, stereoRotation, stereoTranslation,
			nodes.at(0)->rectificationMatrix, nodes.at(1)->rectificationMatrix,
			nodes.at(0)->projectionMatrix, nodes.at(1)->projectionMatrix,
			disparityToDepthMapping, CV_CALIB_ZERO_DISPARITY, 0.0f, imageSize,
			&nodes.at(0)->validRegionOfInterest, &nodes.at(1)->validRegionOfInterest);

	LOG(2, "calibration done!");
}



void StereoCalibration::rectify(cv::Size imageSize, bool addGridOverlay)
{
	this->imageSize = imageSize;
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

