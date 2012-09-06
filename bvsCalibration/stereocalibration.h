#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include<string>
#include<thread>
#include<vector>

#include "calibrationnode.h"
#include "bvs/logger.h"
#include "opencv2/opencv.hpp"



/** Stereo calibration system.
 * This calibrates a stereo camera system.
 */
class StereoCalibration
{
	public:
		/** Constructor.
		 * @param[in] nodes Vector of calibration nodes (MUST be of size 2).
		 */
		StereoCalibration(CalNodeVec& nodes);

		/** Loads calibration from file.
		 * @param[in] path Path to file (directory).
		 * @param[in] file File name.
		 * @return True if file loaded successfully, false otherwise.
		 */
		bool loadFromFile(const std::string& path, const std::string& file);

		/** Save calibration to file.
		 * @param[in] path Path to file (directory).
		 * @param[in] file File name.
		 * @return True if file saved successfully, false otherwise.
		 */
		bool saveToFile(const std::string& path, const std::string& file);

		/** Calibrate nodes.
		 * @param[in] numImages Number of images for calibration.
		 * @param[in] imageSize Size of image where points were detected.
		 * @param[in] boardSize Size of the calibration pattern.
		 * @param[in] blobSize Size of pattern blobs (squares or distance of circle centers).
		 */
		void calibrate(int numImages, cv::Size imageSize, cv::Size boardSize, float blobSize);

		/** Rectify output image.
		 * @param[in] addGridOverlay Whether to overlay a grid on rectified images.
		 */
		void rectify(bool addGridOverlay = false);

	private:
		BVS::Logger logger; /**< Logger instance. */
		CalNodeVec& nodes; /**< Vector of calibration nodes. */
		cv::Size imageSize; /**< Image size. */
		double rms; /** Reprojection error. */
		double averageError; /** Calculated average error. */
		/** Pattern object points (in 3-dimensional coordinates). */
		std::vector<std::vector<cv::Point3f>> objectPoints;
		cv::Mat stereoRotation; /**< Rotation between principal planes. */
		cv::Mat stereoTranslation; /**< Translation between principal planes. */
		/** Essential matrix, relates corresponding stereo points (for pinhole camera model). */
		cv::Mat stereoEssential;
		/** Fundamental matrix, relates corresponding stereo points (for epipolar geometry). */
		cv::Mat stereoFundamental;
		/** Perspective transformation matrix. */
		cv::Mat disparityToDepthMapping;
		/** Rectification map for 2 images. */
		cv::Mat rectifyMap[2][2];
};



#endif // STEREOCALIBRATION_H

