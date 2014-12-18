#include "stereocalibration.h"
#include "bvs/utils.h"
#include "opencv2/opencv.hpp"
#include <iostream>



StereoCalibration::StereoCalibration(CalNodeVec& nodes, bool fisheye)
	: logger("StereoCalib")
	, nodes(nodes)
	, fisheye(fisheye)
	, imageSize()
	, rms(0)
	, initRectifyMap(true)
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

	fs["rms"] >> rms;
	fs["stereoRotation"] >> stereoRotation;
	fs["stereoTranslation"] >> stereoTranslation;
	fs["stereoEssential"] >> stereoEssential;
	fs["stereoFundamental"] >> stereoFundamental;
	fs["disparityToDepthMapping"] >> disparityToDepthMapping;

	for (auto& node: nodes) {
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

	for (auto& node: nodes) {
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



void StereoCalibration::calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, std::string pattern, float blobSize)
{
	if (nodes.size()!=2) {
		std::cerr << "Size of input nodes vector is not 2!" << std::endl;
		exit(1);
	}
	this->imageSize = imageSize;

	// generate a priori object points of detection pattern
	objectPoints.resize(numImages);
	for (int i=0; i<numImages; i++)
		for (int j=0; j<boardSize.height; j++)
			for (int k=0; k<boardSize.width; k++)
				if (pattern.at(0)=='A')
					objectPoints.at(i).push_back(cv::Point3f(double((2*k+j%2)*blobSize/2.), double(j*blobSize/2.), 0.)); // for ASYMMETRIC CIRCLE patterns
				else
					objectPoints.at(i).push_back(cv::Point3f(j*blobSize, k*blobSize, 0)); // for CHESSBOARDS or SYMMETRIC CIRCLE patterns only

	LOG(2, "calibrating individual cameras intrinsics!");
	std::vector<std::thread> threads;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	for (auto& node: nodes) {
		threads.push_back(std::thread([&]{
					BVS::nameThisThread("calIntrins");
					double calError;
					if (fisheye)
						calError = cv::fisheye::calibrate(objectPoints, node->pointStore,
							imageSize, node->cameraMatrix, node->distCoeffs, rvecs, tvecs,
							// TODO: cv::CALIB_RECOMPUTE_EXTRINSIC, cv::CALIB_CHECK_COND, cv::CALIB_FIX_SKEW, cv::CALIB_FIX_K1...K4 <- cv::fisheye::
							cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO +
							cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_SAME_FOCAL_LENGTH +
							cv::CALIB_RATIONAL_MODEL +
							cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
							cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5)
							);
					else
						calError = cv::calibrateCamera(objectPoints, node->pointStore,
							imageSize, node->cameraMatrix, node->distCoeffs, rvecs, tvecs,
							cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO +
							cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_SAME_FOCAL_LENGTH +
							cv::CALIB_RATIONAL_MODEL +
							cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
							cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5)
							);
					LOG(1, "reprojection error for node " << node->id << ": " << calError);
					}));
	}
	for (auto& t: threads) t.join();

	LOG(2, "calibrating stereo!");
	if (fisheye)
		rms = cv::fisheye::stereoCalibrate(
				objectPoints, nodes.at(0)->pointStore, nodes.at(1)->pointStore,
				nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs, nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
				imageSize, stereoRotation, stereoTranslation, // stereoEssential, stereoFundamental,
				// TODO: cv::CALIB_FIX_INTRINSIC or cv::CALIB_USE_INTRINSIC_GUESS, cv::CALIB_RECOMPUTE_EXTRINSIC, cv::CALIB_CHECK_COND, cv::CALIB_FIX_SKEW, cv::CALIB_FIX_K1...K4 <- cv::fisheye::
				cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO +
				cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
				cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5));
	else
		rms = cv::stereoCalibrate(
				objectPoints, nodes.at(0)->pointStore, nodes.at(1)->pointStore,
				nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs, nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
				imageSize, stereoRotation, stereoTranslation, stereoEssential, stereoFundamental,
				cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_ASPECT_RATIO +
				cv::CALIB_SAME_FOCAL_LENGTH + cv::CALIB_RATIONAL_MODEL + cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
				cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 100, 1e-5));
	LOG(1, "stereo reprojection error: " << rms);

	LOG(2, "calculating stereo rectification!");
	if (fisheye)
		cv::fisheye::stereoRectify(
				nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs,
				nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
				imageSize, stereoRotation, stereoTranslation,
				nodes.at(0)->rectificationMatrix, nodes.at(1)->rectificationMatrix,
				nodes.at(0)->projectionMatrix, nodes.at(1)->projectionMatrix,
				disparityToDepthMapping, cv::CALIB_ZERO_DISPARITY, imageSize,
				0.0, 1.0); // TODO: check settings
	else
		cv::stereoRectify(
				nodes.at(0)->cameraMatrix, nodes.at(0)->distCoeffs,
				nodes.at(1)->cameraMatrix, nodes.at(1)->distCoeffs,
				imageSize, stereoRotation, stereoTranslation,
				nodes.at(0)->rectificationMatrix, nodes.at(1)->rectificationMatrix,
				nodes.at(0)->projectionMatrix, nodes.at(1)->projectionMatrix,
				disparityToDepthMapping, cv::CALIB_ZERO_DISPARITY, 0.0, imageSize,
				&nodes.at(0)->validRegionOfInterest, &nodes.at(1)->validRegionOfInterest);

	LOG(2, "calibration done!");
}



void StereoCalibration::rectify(cv::Size imageSize, bool addGridOverlay)
{
	this->imageSize = imageSize;

	if (initRectifyMap) {
		if (fisheye)
			for (int i=0; i<2; i++)
				cv::fisheye::initUndistortRectifyMap(nodes.at(i)->cameraMatrix, nodes.at(i)->distCoeffs,
						nodes.at(i)->rectificationMatrix, nodes.at(i)->projectionMatrix, imageSize,
						CV_16SC2, rectifyMap[i][0], rectifyMap[i][1]);
		else
			for (int i=0; i<2; i++)
				cv::initUndistortRectifyMap(nodes.at(i)->cameraMatrix, nodes.at(i)->distCoeffs,
						nodes.at(i)->rectificationMatrix, nodes.at(i)->projectionMatrix, imageSize,
						CV_16SC2, rectifyMap[i][0], rectifyMap[i][1]);
		initRectifyMap = false;
	}

	for (int i=0; i<2; i++)
		cv::remap(nodes.at(i)->frame, *nodes.at(i)->output, rectifyMap[i][0], rectifyMap[i][1], cv::INTER_LINEAR);

	if (addGridOverlay) {
		for (auto& node: nodes) {
			for(int i = 0; i < node->output->rows; i += node->output->rows/10)
				cv::line(*node->output, cv::Point(0, i), cv::Point(node->output->cols, i), cv::Scalar(0, 255, 0), 1, 8);
			for(int i = 0; i < node->output->cols; i += node->output->cols/10)
				cv::line(*node->output, cv::Point(i, 0), cv::Point(i, node->output->rows), cv::Scalar(0, 255, 0), 1, 8);
		}
	}
}

