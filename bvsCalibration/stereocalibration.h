#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include<string>
#include<thread>
#include<vector>

#include "calibrationnode.h"
#include "bvs/logger.h"
#include "opencv2/opencv.hpp"



//TODO comments
class StereoCalibration
{
	public:
		StereoCalibration(CalNodeVec& nodes);

		bool loadFromFile(const std::string& path, const std::string& file);
		bool saveToFile(const std::string& path, const std::string& file);

		void calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, float circleSize);
		void rectify(bool addGridOverlay = false);
	private:
		BVS::Logger logger;

		CalNodeVec& nodes;

		cv::Size imageSize;
		double rms;
		double averageError;

		std::vector<std::vector<cv::Point3f>> objectPoints;
		cv::Mat stereoRotation;
		cv::Mat stereoTranslation;
		cv::Mat stereoEssential;
		cv::Mat stereoFundamental;

		cv::Mat disparityToDepthMapping;

		cv::Mat rectifyMap[2][2];
};

#endif // STEREOCALIBRATION_H


