#ifndef CALIBRATION_NODE_H
#define CALIBRATION_NODE_H

#include<vector>

#include "bvs/connector.h"
#include "opencv2/opencv.hpp"



// forward declaration
struct CalibrationNode;



/** Calibration Node vector definition. */
typedef std::vector<CalibrationNode*> CalNodeVec;



/** Calibration node meta data. */
struct CalibrationNode
{
	CalibrationNode( int id,
	BVS::Connector<cv::Mat> input, /**< Input connector. */
	BVS::Connector<cv::Mat> output, /**< Output connector. */
	cv::Mat frame, /**< Input frame. */
	cv::Mat scaledFrame, /**< Scaled input frame. */
	std::vector<cv::Point2f> framePoints, /**< Current frame points. */
	cv::Mat sample, /**< Full size sample. */
	std::vector<cv::Point2f> points, /**< Detected points. */
	std::vector<std::vector<cv::Point2f>> pointStore, /**< Store of detected points. */
	cv::Mat cameraMatrix, /**< Internal camera matrix. */
	cv::Mat distCoeffs, /**< Distortion coefficients. */
	cv::Mat rectificationMatrix, /**< Rectification matrix. */
	cv::Mat projectionMatrix /**< Projection matrix. */
	)
		:
		id(id),
		input(input),
		output(output),
		frame(frame),
		scaledFrame(scaledFrame),
		framePoints(framePoints),
		sample(sample),
		points(points),
		pointStore(pointStore),
		cameraMatrix(cameraMatrix),
		distCoeffs(distCoeffs),
		rectificationMatrix(rectificationMatrix),
		projectionMatrix(projectionMatrix)
		{}

	int id; /**< Identifier. */
	BVS::Connector<cv::Mat> input; /**< Input connector. */
	BVS::Connector<cv::Mat> output; /**< Output connector. */
	cv::Mat frame; /**< Input frame. */
	cv::Mat scaledFrame; /**< Scaled input frame. */
	std::vector<cv::Point2f> framePoints; /**< Current frame points. */
	cv::Mat sample; /**< Full size sample. */
	std::vector<cv::Point2f> points; /**< Detected points. */
	std::vector<std::vector<cv::Point2f>> pointStore; /**< Store of detected points. */
	cv::Mat cameraMatrix; /**< Internal camera matrix. */
	cv::Mat distCoeffs; /**< Distortion coefficients. */
	cv::Mat rectificationMatrix; /**< Rectification matrix. */
	cv::Mat projectionMatrix; /**< Projection matrix. */
};



#endif // CALIBRATION_NODE_H

