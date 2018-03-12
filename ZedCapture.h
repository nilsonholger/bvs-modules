#ifndef ZEDCAPTURE_H
#define ZEDCAPTURE_H

#include "bvs/module.h"
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

class ZedCapture : public BVS::Module
{
	public:

		ZedCapture(BVS::ModuleInfo info, const BVS::Info& bvs);

		~ZedCapture();

		BVS::Status execute();

		/** UNUSED
		 * @return Module's status.
		 */
		BVS::Status debugDisplay();

	private:

        const BVS::ModuleInfo info;
        BVS::Logger logger;
        const BVS::Info& bvs;

        bool mConfShowImages;
        bool mConfWithDepth;
        bool mConfWithPointCloud;
        bool mConfWithTracking;
        std::string mConfCamResolution;
        int mConfFps;
        std::string mConfDepthMode;
        std::string mConfDepthQuality;
        std::string mConfDepthUnits;
        bool mConfMeasureDepthRight;
        bool mConfWriteToFile;
        std::string mConfOutputDir;
        bool mConfPlaybackRec;
        std::string mConfPathToRec;
        bool mConfWithNormals;

        boost::filesystem::path mOutputPath;

        sl::Camera mCamera;
        sl::RuntimeParameters mRuntimeParameters;
        bool mShutdown;

        int mFrameCounter;

        BVS::Connector<cv::Mat> mOutputImgLeft;
        BVS::Connector<cv::Mat> mOutputImgRight;
        BVS::Connector<cv::Mat> mOutputDepthLeft;
        BVS::Connector<cv::Mat> mOutputDepthRight;
        BVS::Connector<cv::Mat> mOutputPointCloudLeft;
        BVS::Connector<cv::Mat> mOutputPointCloudRight;
        BVS::Connector<cv::Mat> mOutputNormalsLeft;
        BVS::Connector<cv::Mat> mOutputNormalsRight;
        BVS::Connector<cv::Mat> mOutputMotion;

        cv::Mat slMat2cvMat(const sl::Mat& input) const;


		ZedCapture(const ZedCapture&) = delete; /**< -Weffc++ */
		ZedCapture& operator=(const ZedCapture&) = delete; /**< -Weffc++ */
};

#endif //ZEDCAPTURE_H

