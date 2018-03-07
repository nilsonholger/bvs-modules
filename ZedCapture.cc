#include "ZedCapture.h"



ZedCapture::ZedCapture(BVS::ModuleInfo info, const BVS::Info& bvs)
    : BVS::Module()
    , info(info)
    , logger(info.id)
    , bvs(bvs)
    , mConfShowImages(bvs.config.getValue<bool>(info.conf+".showImages", false))
    , mConfCamResolution(bvs.config.getValue<std::string>(info.conf+".cameraResolution", ""))
    , mConfFps(bvs.config.getValue<int>(info.conf+".cameraFps", 60))
    , mConfDepthMode(bvs.config.getValue<std::string>(info.conf+".depthSensingMode", ""))
    , mConfDepthQuality(bvs.config.getValue<std::string>(info.conf+".depthQuality", ""))
    , mCamera()
    , mRuntimeParameters()
    , mShutdown(false)
    , mOutputImgLeft{"imgLeft", BVS::ConnectorType::OUTPUT}
    , mOutputImgRight{"imgRight", BVS::ConnectorType::OUTPUT}
    , mOutputDepthLeft{"depthLeft", BVS::ConnectorType::OUTPUT}
    , mOutputDepthRight{"depthRight", BVS::ConnectorType::OUTPUT}

{
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

    if (mConfFps == 15) {
        initParams.camera_fps = 15;
    } else if (mConfFps == 30) {
        initParams.camera_fps = 30;
    } else {
        initParams.camera_fps = 60;
    }


    if (mConfDepthMode == "FILL") {
        mRuntimeParameters.sensing_mode = sl::SENSING_MODE_FILL;
    } else {
        mRuntimeParameters.sensing_mode = sl::SENSING_MODE_STANDARD;
    }

    if (mConfDepthQuality == "MEDIUM") {
        initParams.depth_mode = sl::DEPTH_MODE_MEDIUM;
    } else if (mConfDepthQuality == "QUALITY") {
        initParams.depth_mode = sl::DEPTH_MODE_QUALITY;
    } else if (mConfDepthQuality == "ULTRA") {
        initParams.depth_mode = sl::DEPTH_MODE_ULTRA;
    } else {
        initParams.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    }

    initParams.coordinate_units = sl::UNIT_METER;




    sl::ERROR_CODE err = mCamera.open(initParams);
    if (err != sl::SUCCESS) {
        mShutdown = true;
    }

}



// This is your module's destructor.
ZedCapture::~ZedCapture() {

}


cv::Mat ZedCapture::slMat2cvMat(sl::Mat& input) const {
    // Mapping between MAT_TYPE and CV_TYPE
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

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}



BVS::Status ZedCapture::execute() {
    if (mShutdown) {
        LOG(3, "Could not open camera");
        return BVS::Status::SHUTDOWN;
    }

    //left camera image
    sl::Mat imageZedLeft(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
    cv::Mat imageOcvLeft = slMat2cvMat(imageZedLeft);

    //right camera image
    sl::Mat imageZedRight(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
    cv::Mat imageOcvRight = slMat2cvMat(imageZedRight);

    //depth left
    sl::Mat depthZedLeft(mCamera.getResolution(), sl::MAT_TYPE_32F_C1);
    cv::Mat depthOcvLeft = slMat2cvMat(depthZedLeft);

    sl::Mat depthImgZedLeft(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
    cv::Mat depthImgOcvLeft = slMat2cvMat(depthImgZedLeft);


    //depth right
    sl::Mat depthZedRight(mCamera.getResolution(), sl::MAT_TYPE_32F_C1);
    cv::Mat depthOcvRight = slMat2cvMat(depthZedRight);

    sl::Mat depthImgZedRight(mCamera.getResolution(), sl::MAT_TYPE_8U_C4);
    cv::Mat depthImgOcvRight = slMat2cvMat(depthImgZedRight);

    if (mCamera.grab(mRuntimeParameters) == sl::SUCCESS) {
        // A new image is available if grab() returns SUCCESS
        mCamera.retrieveImage(imageZedLeft, sl::VIEW_LEFT); // Retrieve the left image
        mCamera.retrieveImage(imageZedRight, sl::VIEW_RIGHT); // Retrieve the right image

        mCamera.retrieveMeasure(depthZedLeft, sl::MEASURE_DEPTH); // Retrieve the left depth image
        mCamera.retrieveImage(depthImgZedLeft, sl::VIEW_DEPTH); // Retrieve the left depth image to display

        mCamera.retrieveMeasure(depthZedRight, sl::MEASURE_DEPTH); // Retrieve the right depth image
        mCamera.retrieveImage(depthImgZedRight, sl::VIEW_DEPTH); // Retrieve the right depth image to display

        if (mConfShowImages) {
            cv::namedWindow("imgLeft", CV_WINDOW_NORMAL);
            cv::imshow("imgLeft", imageOcvLeft);

            cv::namedWindow("imgRight", CV_WINDOW_NORMAL);
            cv::imshow("imgRight", imageOcvRight);

            cv::namedWindow("dispLeft", CV_WINDOW_NORMAL);
            cv::imshow("dispLeft", depthImgOcvLeft);

            cv::namedWindow("dispRight", CV_WINDOW_NORMAL);
            cv::imshow("dispRight", depthImgOcvRight);

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

