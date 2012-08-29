#include "calibrationguide.h"



CalibrationGuide::CalibrationGuide(const int& numImages, const int& numDetections, float scale, int centerDetections, int sectorDetections)
	: numImages(numImages),
	numDetections(numDetections),
	scale(scale),
	centerDetections(centerDetections),
	sectorDetections(sectorDetections),
	state(0),
	imgDiag(0),
	size{0, 0},
	center{0, 0},
	p1{0, 0},
	p2{0, 0},
	detCenter{0, 0},
	color{0, 0, 255}
{ }



void CalibrationGuide::addTargetOverlay(cv::Mat& img)
{
	if (numImages!=centerDetections+8*sectorDetections)
	{
		cv::putText(img, "ERROR!!!",
				cv::Point(10, img.rows/2), CV_FONT_HERSHEY_DUPLEX, 1.0f, color, 2);
		cv::putText(img, "numDetections != centerDetections + 8 * sectorDetections!",
				cv::Point(10, img.rows/2+30), CV_FONT_HERSHEY_DUPLEX, 1.0f, color);
		return;
	}

	float border;
	if (img.cols != size.x || img.rows != size.y) updateInputSize(img);

	switch (state)
	{
		case 0:
			border = (1.0f-scale)/2.0f;
			p1 = {img.cols*border, img.rows*border};
			p2 = {img.cols*(1.0f-border), img.rows*(1.0f-border)};
			center = {size.x/2, size.y/2};
			break;
		case 1: updateTarget(0, 0); break;
		case 2: updateTarget(1, 0); break;
		case 3: updateTarget(2, 0); break;
		case 4: updateTarget(2, 1); break;
		case 5: updateTarget(2, 2); break;
		case 6: updateTarget(1, 2); break;
		case 7: updateTarget(0, 2); break;
		case 8: updateTarget(0, 1); break;
	}

	cv::rectangle(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), color, 2);
}



bool CalibrationGuide::checkDetectionQuality(cv::Mat& img, std::vector<cv::Point2f>& points)
{
	int diag = std::sqrt(std::pow(points.back().x-points.front().x, 2)
			+ std::pow(points.back().y-points.front().y, 2));
	detCenter.x = (points.front().x+points.back().x)/2;
	detCenter.y = (points.front().y+points.back().y)/2;

	switch (state)
	{
		case 0:
			cv::rectangle(img, cv::Point(p2.x, p2.y), cv::Point(p2.x-10, p1.y+(p2.y-p1.y)*(1.0f-(float)diag/(float)imgDiag/scale)),
					color, CV_FILLED);
			if (diag < imgDiag*scale)
			{
				color = {0, 0, 255};
				cv::putText(img, "Detected pattern size TO SMALL, MOVE CLOSER.",
						cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
						1.0f, color);
				return false;
			}
			else cv::putText(img, "Detected pattern size OK.",
					cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
					1.0f, color);
			if (numDetections >= centerDetections) state++;
			break;
		case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
			cv::line(img, cv::Point(center.x, center.y), cv::Point(detCenter.x, detCenter.y), color, 2);
			if (numDetections < centerDetections+sectorDetections*(state-1)) state--;
			if (diag > imgDiag/2 || std::abs(detCenter.x-center.x)>size.x/6 || std::abs(detCenter.y-center.y)>size.y/6)
			{
				color = {0, 0, 255};
				cv::putText(img, "Pattern center position OUTSIDE OF TARGET.",
						cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
						1.0f, color);
				return false;
			}
			else cv::putText(img, "Pattern center position OK.",
					cv::Point(10, img.rows-10), CV_FONT_HERSHEY_DUPLEX,
					1.0f, color);
			if (numDetections >= centerDetections+sectorDetections*state) state++;
			break;
		default:
			return false;
	}

	color = {0, 255, 0};

	return true;
}



void CalibrationGuide::reorderDetections(std::vector<std::vector<cv::Point2f>>& points)
{
	std::vector<std::vector<cv::Point2f>> tmp;

	for (int i=0; i<sectorDetections; i++)
	{
		tmp.push_back(points.at(centerDetections+i));
		tmp.push_back(points.at(centerDetections+i+3*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+6*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+1*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+4*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+7*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+2*sectorDetections));
		tmp.push_back(points.at(centerDetections+i+5*sectorDetections));
	}

	points.erase(points.begin()+centerDetections, points.end());
	points.insert(points.begin()+centerDetections, tmp.begin(), tmp.end());
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

