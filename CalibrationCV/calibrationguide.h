#ifndef CALIBRATION_GUIDE_H
#define CALIBRATION_GUIDE_H

#include<vector>
#include "opencv2/opencv.hpp"



/** Calibration Guidance System.
 * This is a guidance system to improve the quality of calibration images, or
 * to be more precise, the position and size of the detected patterns. The
 * purpose of this is to increase absolute pixel coverage by the detected
 * patterns.
 * TODO redesign: enforce different size detections spread evenly over the image
 */
class CalibrationGuide
{
	public:
		/** CalibrationGuide constructor.
		 * @param[in] numImages The overall number of images.
		 * This should be centerDetections + 8 * sectorDetections.
		 * @param[in] numDetections The number of pattern detections so far.
		 * @param[in] sectorDetections Number of per sector detections.
		 */
		CalibrationGuide(const int& numImages, const int& numDetections, int sectorDetections);

		/** Draws target information.
		 * This method draws target information on an image. These consists of a
		 * rectangle of the desired size and position.
		 * @param[in] img The image to draw on.
		 */
		void addTargetOverlay(cv::Mat& img);

		/** Checks detected pattern quality.
		 * Given an image and the detected points, this method checks the quality
		 * of the detection. It does so by calculating the patterns diagonal and
		 * center position and comparing them with the requirements.
		 * It also draws a line onto the image showing the distance between the
		 * center of the detected pattern and the desired position.
		 * @param[in] img The image to draw on.
		 * @param[in] points Vector of detected pattern points.
		 * @return True if the detection satisfies all conditions, false otherwise.
		 */
		bool checkDetectionQuality(cv::Mat& img, std::vector<cv::Point2f>& points);

	private:
		/** Update input size.
		 * @param[in] img Image to retrieve size from.
		 */
		void updateInputSize(cv::Mat& img);

		/** Modifies the target position.
		 *
		 * @code
		 * sector: XY
		 * state: n
		 *
		 *   00  |  10  |  20
		 *    0  |   1  |   2
		 * --------------------
		 *   01  |  11  |  21
		 *    5  |   4  |   3
		 * --------------------
		 *   02  |  12  |  22
		 *    6  |   7  |   8
		 * @endcode
		 *
		 * @param[in] sectorX X coordinate of sector.
		 * @param[in] sectorY Y coordinate of sector.
		 */
		void updateTarget(int sectorX, int sectorY);

		const int& numImages; /**< Overall number of images used for calibration. */
		const int& numDetections; /**< Number of actual detections. */
		int sectorDetections; /**< Number of per sector detections. */

		/** Detection state.
		 * @li \c 0: large center detections until numDetections = centerDetections,
		 * check if size of detection > targetSize
		 * @li \c 1-8: sector detections until numDetections = centerDetections and
		 * state * sectorDetections, check if size of detection > (targetSize/3) and
		 * if center close to sector center
		 */
		int state;
		double imgDiag; /**< Lenght of image diagonal. */

		/** Point structure. */
		struct point
		{
			float x; /**< X position. */
			float y; /**< Y position. */
		};

		point size; /**< Size of image. */
		point center; /**< Target center. */
		point p1; /**< Upper left target point. */
		point p2; /**< Lower right target point. */
		point detCenter; /**< Detection center. */
		cv::Scalar color; /**< Target rectangle color. */
};



#endif // CALIBRATION_GUIDE_H

