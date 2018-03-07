#include "ZedCapture.h"
#include <opencv2/imgproc.hpp>



ZedCapture::ZedCapture(BVS::ModuleInfo info, const BVS::Info& bvs)
    : BVS::Module()
    , info(info)
    , logger(info.id)
    , bvs(bvs)
    , mConfShowImages(bvs.config.getValue<bool>(info.conf+".showImages", false))
    , mConfWithDepth(bvs.config.getValue<bool>(info.conf+".withDepth", true))
    , mConfWithPointCloud(bvs.config.getValue<bool>(info.conf+".withPointCloud", true))
    , mConfWithTracking(bvs.config.getValue<bool>(info.conf+".withTracking", true))
    , mConfCamResolution(bvs.config.getValue<std::string>(info.conf+".cameraResolution", ""))
    , mConfFps(bvs.config.getValue<int>(info.conf+".cameraFps", 60))
    , mConfDepthMode(bvs.config.getValue<std::string>(info.conf+".depthSensingMode", ""))
    , mConfDepthQuality(bvs.config.getValue<std::string>(info.conf+".depthQuality", ""))
    , mConfDepthUnits(bvs.config.getValue<std::string>(info.conf+".depthUnits", ""))
    , mConfMeasureDepthRight(bvs.config.getValue<bool>(info.conf+".measureDepthRight", false))
    , mCamera()
    , mRuntimeParameters()
    , mShutdown(false)
    , mOutputImgLeft{"imgLeft", BVS::ConnectorType::OUTPUT}
    , mOutputImgRight{"imgRight", BVS::ConnectorType::OUTPUT}
    , mOutputDepthLeft{"depthLeft", BVS::ConnectorType::OUTPUT}
    , mOutputDepthRight{"depthRight", BVS::ConnectorType::OUTPUT}
    , mOutputPointCloudLeft{"cloudLeft", BVS::ConnectorType::OUTPUT}
    , mOutputPointCloudRight{"cloudRight", BVS::ConnectorType::OUTPUT}
    , mOutputMotion{"motion", BVS::ConnectorType::OUTPUT}

{


    //resolution
    sl::InitParameters initParams;

    if (mConfCamResolution == "VGA") {
        initParams.camera_resolution = sl::RESOLUTION_VGA;
    } else if (mConfCamResolution == "HD1080") {
        initParams.camera_resolution = sl::RESOLUTION_HD1080;
    } else if (mConfCamResolution == "HD2K") {
        initParams.camera_resolution = sl::RESOLUTION_HD2K;
    } else {
        initParams.camera_resolution = sl::RESOLUTION_HD720;
    }

    //fps
    if (mConfFps == 15) {
        initParams.camera_fps = 15;
    } else if (mConfFps == 30) {
        initParams.camera_fps = 30;
    } else {
        initParams.camera_fps = 60;
    }

    //depth sensing mode
    if (mConfDepthMode == "FILL") {
        mRuntimeParameters.sensing_mode = sl::SENSING_MODE_FILL;
    } else {
        mRuntimeParameters.sensing_mode = sl::SENSING_MODE_STANDARD;
    }

    mRuntimeParameters.enable_depth = mConfWithDepth;
    mRuntimeParameters.enable_point_cloud = mConfWithPointCloud;

    //depth quality
    if (mConfDepthQuality == "MEDIUM") {
        initParams.depth_mode = sl::DEPTH_MODE_MEDIUM;
    } else if (mConfDepthQuality == "QUALITY") {
        initParams.depth_mode = sl::DEPTH_MODE_QUALITY;
    } else if (mConfDepthQuality == "ULTRA") {
        initParams.depth_mode = sl::DEPTH_MODE_ULTRA;
    } else {
        initParams.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    }

    //depth units)
    if (mConfDepthUnits == "MM") {
        initParams.coordinate_units = sl::UNIT_MILLIMETER;
    } else if (mConfDepthUnits == "CM") {
        initParams.coordinate_units = sl::UNIT_CENTIMETER;
    } else if (mConfDepthUnits == "INCHES") {
        initParams.coordinate_units = sl::UNIT_INCH;
    } else if (mConfDepthUnits == "FEET") {
        initParams.coordinate_units = sl::UNIT_FOOT;
    } else {
        initParams.coordinate_units = sl::UNIT_METER;
    }

    //measure depth right
    if (mConfMeasureDepthRight) {
        initParams.enable_right_side_measure = true;
    } else {
        initParams.enable_right_side_measure = false;
    }



    sl::ERROR_CODE err = mCamera.open(initParams);
    if (err != sl::SUCCESS) {
        LOG(3, "Could not open camera");
        mShutdown = true;
    } else {
        if (mConfWithTracking) {
            sl::TrackingParameters trackingParams;
            err = mCamera.enableTracking(trackingParams);
            if (err != sl::SUCCESS) {
                LOG(3, "Could not enable tracking");
                mShutdown = true;
            }
        }
    }



}



// This is your module's destructor.
ZedCapture::~ZedCapture() {

}


cv::Mat ZedCapture::slMat2cvMat(sl::Mat& input) const {
    int cv_type = -1;
    switch (input.getDataType()) {
    case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
    default: break;
    }

    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}



BVS::Status ZedCapture::execute() {
    if (mShutdown) {

        return BVS::Status::SHUTDOWN;
    }

    if (mCamera.grab(mRuntimeParameters) == sl::SUCCESS) {
        //left camera image
        sl::Mat imageZedLeft(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
        cv::Mat imageOcvLeft = slMat2cvMat(imageZedLeft);

        //right camera image
        sl::Mat imageZedRight(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
        cv::Mat imageOcvRight = slMat2cvMat(imageZedRight);

        mCamera.retrieveImage(imageZedLeft, sl::VIEW_LEFT); // Retrieve the left image
        mCamera.retrieveImage(imageZedRight, sl::VIEW_RIGHT); // Retrieve the right image

        cv::Mat imgLeftNoAlpha;
        cv::cvtColor(imageOcvLeft, imgLeftNoAlpha, cv::COLOR_BGRA2GRAY);
        mOutputImgLeft.send(imgLeftNoAlpha);

        cv::Mat imgRightNoAlpha;
        cv::cvtColor(imageOcvRight, imgRightNoAlpha, cv::COLOR_BGRA2GRAY);
        mOutputImgRight.send(imgRightNoAlpha);

        if (mConfShowImages) {
            cv::namedWindow("imgLeft", CV_WINDOW_NORMAL);
            cv::imshow("imgLeft", imgLeftNoAlpha);

            cv::namedWindow("imgRight", CV_WINDOW_NORMAL);
            cv::imshow("imgRight", imgRightNoAlpha);
        }

        if (mConfWithDepth) {
            //depth left
            sl::Mat depthZedLeft(mCamera.getResolution(), sl::MAT_TYPE_32F_C1);
            cv::Mat depthOcvLeft = slMat2cvMat(depthZedLeft);

            sl::Mat depthImgZedLeft(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
            cv::Mat depthImgOcvLeft = slMat2cvMat(depthImgZedLeft);

            mCamera.retrieveMeasure(depthZedLeft, sl::MEASURE_DEPTH); // Retrieve the left depth image
            mCamera.retrieveImage(depthImgZedLeft, sl::VIEW_DEPTH); // Retrieve the left depth image to display

            if (mConfShowImages) {
                cv::namedWindow("depthLeft", CV_WINDOW_NORMAL);
                cv::imshow("depthLeft", depthImgOcvLeft);
            }

            mOutputDepthLeft.send(depthOcvLeft);

            if (mConfMeasureDepthRight) {
                //depth right
                sl::Mat depthZedRight(mCamera.getResolution(), sl::MAT_TYPE_32F_C1);
                cv::Mat depthOcvRight = slMat2cvMat(depthZedRight);

                sl::Mat depthImgZedRight(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
                cv::Mat depthImgOcvRight = slMat2cvMat(depthImgZedRight);

                mCamera.retrieveMeasure(depthZedRight, sl::MEASURE_DEPTH_RIGHT); // Retrieve the right depth image
                mCamera.retrieveImage(depthImgZedRight, sl::VIEW_DEPTH_RIGHT); // Retrieve the right depth image to display

                if (mConfShowImages) {
                    cv::namedWindow("depthRight", CV_WINDOW_NORMAL);
                    cv::imshow("depthRight", depthImgOcvRight);
                }

                mOutputDepthRight.send(depthOcvRight);
            }
        }

        if (mConfWithPointCloud) {
            //point cloud left
            sl::Mat pointCloudZedLeft(mCamera.getResolution(), sl::MAT_TYPE_32F_C4);
            cv::Mat pointCloudOcvLeft = slMat2cvMat(pointCloudZedLeft);

            mCamera.retrieveMeasure(pointCloudZedLeft, sl::MEASURE_XYZRGBA); //Retrieve the left point cloud

            mOutputPointCloudLeft.send(pointCloudOcvLeft);

            if (mConfMeasureDepthRight) {
                //point cloud right
                sl::Mat pointCloudZedRight(mCamera.getResolution(), sl::MAT_TYPE_32F_C4);
                cv::Mat pointCloudOcvRight = slMat2cvMat(pointCloudZedRight);

                mCamera.retrieveMeasure(pointCloudZedRight, sl::MEASURE_XYZRGBA_RIGHT); //Retrieve the right point cloud

                mOutputPointCloudRight.send(pointCloudOcvRight);
            }
        }

        if (mConfWithTracking) {
            cv::Mat motion(4,4, CV_64FC1, cv::Scalar(0.0));
            sl::Pose pose;
            sl::TRACKING_STATE state = mCamera.getPosition(pose, sl::REFERENCE_FRAME_WORLD);
            if (state == sl::TRACKING_STATE_OK) {
                sl::Translation trans = pose.getTranslation();
                sl::Rotation rot = pose.getRotationMatrix();

                motion.at<double>(0,0) = static_cast<double>(rot.r00);
                motion.at<double>(0,1) = static_cast<double>(rot.r01);
                motion.at<double>(0,2) = static_cast<double>(rot.r02);
                motion.at<double>(0,3) = static_cast<double>(trans.tx);
                motion.at<double>(1,0) = static_cast<double>(rot.r10);
                motion.at<double>(1,1) = static_cast<double>(rot.r11);
                motion.at<double>(1,2) = static_cast<double>(rot.r12);
                motion.at<double>(1,3) = static_cast<double>(trans.ty);
                motion.at<double>(2,0) = static_cast<double>(rot.r20);
                motion.at<double>(2,1) = static_cast<double>(rot.r21);
                motion.at<double>(2,2) = static_cast<double>(rot.r22);
                motion.at<double>(2,3) = static_cast<double>(trans.tz);
                motion.at<double>(3,0) = 0.0;
                motion.at<double>(3,1) = 0.0;
                motion.at<double>(3,2) = 0.0;
                motion.at<double>(3,3) = 1.0;

                mOutputMotion.send(motion);

            } else if (state == sl::TRACKING_STATE_SEARCHING) {
                //TODO: check whether tracking is lost and reset accordingly
            }

        }

        if (mConfShowImages) {
            cv::waitKey(1);
        }

    }

    return BVS::Status::OK;
}



// UNUSED
BVS::Status ZedCapture::debugDisplay() {
    return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(ZedCapture)

