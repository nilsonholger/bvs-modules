#ifndef CALIBRATION_NODE_H
#define CALIBRATION_NODE_H

#include<vector>
#include "bvs/connector.h"
#include "opencv2/opencv.hpp"



// Forward declaration.
struct CalibrationNode;



/** Calibration Node vector definition. */
typedef std::vector<CalibrationNode*> CalNodeVec;



/** Calibration node meta data. */
struct CalibrationNode
{
	/** CalibrationNode Constructor.
	 * @param[in] id Identifier.
	 * @param[in] input Input connector.
	 * @param[in] output Output connector.
	 * @param[in] frame Input frame.
	 * @param[in] scaledFrame Scaled version of input frame.
	 * @param[in] framePoints Detected pattern points in scaled frame.
	 * @param[in] sample Full size sample copy for detection.
	 * @param[in] points Detected pattern points in sample.
	 * @param[in] pointStore Vector of detected points.
	 * @param[in] cameraMatrix Camera intrinsics.
	 * @param[in] distCoeffs Distortion coefficients.
	 * @param[in] rectificationMatrix Rectification matrix.
	 * @param[in] projectionMatrix Projection matrix.
	 * @param[in] homographyMatrix Homography matrix.
	 * @param[in] validRegionOfInterest Calculated valid region of interest.
	 */
	CalibrationNode(int id, BVS::Connector<cv::Mat> input, BVS::Connector<cv::Mat> output,
			cv::Mat frame, cv::Mat scaledFrame, std::vector<cv::Point2f> framePoints, cv::Mat sample,
			std::vector<cv::Point2f> points, std::vector<std::vector<cv::Point2f>> pointStore,
			cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat rectificationMatrix, cv::Mat projectionMatrix,
			cv::Mat homographyMatrix, cv::Rect validRegionOfInterest)
		: id(id),
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
		projectionMatrix(projectionMatrix),
		homographyMatrix(homographyMatrix),
		validRegionOfInterest(validRegionOfInterest)
	{ }

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
	cv::Mat homographyMatrix; /**< Homography matrix. */
	cv::Rect validRegionOfInterest; /**< Calculated valid region of interest. */
};



#endif // CALIBRATION_NODE_H

