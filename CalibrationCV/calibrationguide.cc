#include "calibrationguide.h"



CalibrationGuide::CalibrationGuide(const int& numImages, const int& numDetections, int sectorDetections)
	: numImages(numImages)
	, numDetections(numDetections)
	, sectorDetections(sectorDetections)
	, state(0)
	, imgDiag(0)
	, size{0, 0}
	, center{0, 0}
	, p1{0, 0}
	, p2{0, 0}
	, detCenter{0, 0}
	, color{0, 0, 255}
{ }



void CalibrationGuide::addTargetOverlay(cv::Mat& img)
{
	// check correct settings usage
	if (numImages!=9*sectorDetections) {
		cv::putText(img, "ERROR!!!",
				cv::Point(10, img.rows/2), CV_FONT_HERSHEY_DUPLEX, 1.0f, color, 2);
		cv::putText(img, "numImages != 9 * sectorDetections!",
				cv::Point(10, img.rows/2+30), CV_FONT_HERSHEY_DUPLEX, 1.0f, color);
		return;
	}

	if (img.cols != size.x || img.rows != size.y) updateInputSize(img);

	// update target rectangle
	switch (state) {
		case 0: updateTarget(0, 0); break;
		case 1: updateTarget(1, 0); break;
		case 2: updateTarget(2, 0); break;
		case 3: updateTarget(2, 1); break;
		case 4: updateTarget(1, 1); break;
		case 5: updateTarget(0, 1); break;
		case 6: updateTarget(0, 2); break;
		case 7: updateTarget(1, 2); break;
		case 8: updateTarget(2, 2); break;
	}
	cv::rectangle(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), color, 2);
}



bool CalibrationGuide::checkDetectionQuality(cv::Mat& img, std::vector<cv::Point2f>& points)
{
	// calculate pattern position
	//double diag = std::sqrt(std::pow(points.back().x-points.front().x, 2)
			//+ std::pow(points.back().y-points.front().y, 2));
	detCenter.x = (points.front().x+points.back().x)/2;
	detCenter.y = (points.front().y+points.back().y)/2;

	// display quality
	// TODO use for scale enforcement
	//cv::rectangle(img, cv::Point(p2.x, p2.y),
			//cv::Point(p2.x-10, p1.y+(p2.y-p1.y)*(1.-diag/imgDiag)),
			//color, CV_FILLED);
	cv::line(img, cv::Point(center.x, center.y), cv::Point(detCenter.x, detCenter.y), color, 2);

	// display advice
	if (/*diag > imgDiag/2 ||*/ std::abs(detCenter.x-center.x)>size.x/6 || std::abs(detCenter.y-center.y)>size.y/6) {
		color = {0, 0, 255};
		cv::putText(img, "Pattern center position OUTSIDE OF TARGET.",
				cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
				1.0f, color);
		return false;
	} else cv::putText(img, "Pattern center position OK.",
			cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
			1.0f, color);

	// update state
	if (numDetections < sectorDetections*state) state--;
	if (numDetections >= sectorDetections*(state+1)) state++;

	color = {0, 255, 0};

	return true;
}



void CalibrationGuide::updateInputSize(cv::Mat& img)
{
	size.x = img.cols;
	size.y = img.rows;
	imgDiag = std::sqrt(size.x*size.x + size.y*size.y);
}



void CalibrationGuide::updateTarget(int sectorX, int sectorY)
{
	p1 = {sectorX*size.x/3, sectorY*size.y/3};
	p2 = {(sectorX+1)*size.x/3, (sectorY+1)*size.y/3};
	center = {(p2.x+p1.x)/2, (p2.y+p1.y)/2};
}

