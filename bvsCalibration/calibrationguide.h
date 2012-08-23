#ifndef CALIBRATION_GUIDE_H
#define CALIBRATION_GUIDE_H

#include<vector>

#include "opencv2/opencv.hpp"


/**
 * TODO show usage information on startup
 *
 * sector/state relationships:
 * @code
 *   00  |  10  |  20
 *    1  |   2  |   3
 * --------------------
 *   01  |  11  |  21
 *    8  |   0  |   4
 * --------------------
 *   02  |  12  |  22
 *    7  |   6  |   5
 * @endcode
 *
 */
class CalibrationGuide
{
	public:
		CalibrationGuide(int& numImages, int& numDetections, float scale, int centerDetections, int sectorDetections);

		void addTargetOverlay(cv::Mat& img);
		bool checkDetectionQuality(cv::Mat& img, std::vector<cv::Point2f>& points);
		void reorderDetections(std::vector<std::vector<cv::Point2f>>& points);

	private:
		void updateInputSize(cv::Mat& img);
		void updateTarget(int sectorX, int sectorY);

		int& numImages;
		int& numDetections;
		float scale;
		int centerDetections;
		int sectorDetections;
		int state;
		int imgDiag;

		struct point
		{
			float x;
			float y;
		};

		point size;
		point center;
		point p1;
		point p2;
		point detCenter;
		cv::Scalar color;
		//TODO
		// check in module or set: numImages = centerDetections + sectorDetections*8
		// option: reorder detections -> center -> alternate between sectors in each detection
		// functions need to act depending on state:
		// state 0: large center until numDetections = centerDetections, state++;
		//		check: size of detection > targetSize
		// state 1-8: sector until numDetections = centerDetections + state * sectorDetections, state++
		//		check: size of detection > (targetSize/3) and center close to sector center
		// state 9: signal done
		//
		// add function to reorder detections to improve calibration distribution
};



#endif // CALIBRATION_GUIDE_H
