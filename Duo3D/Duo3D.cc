#include "Duo3D.h"



PDUOFrame Duo3D::duo_frame = nullptr;
std::mutex Duo3D::mutex{};



Duo3D::Duo3D(BVS::ModuleInfo info, const BVS::Info& _bvs)
	: BVS::Module{}
	, info(info)
	, logger{info.id}
	, bvs{_bvs}
	, outL{"outL", BVS::ConnectorType::OUTPUT}
	, outR{"outR", BVS::ConnectorType::OUTPUT}
	, outTime{"outTime", BVS::ConnectorType::OUTPUT}
	, outAccel{"outAccel", BVS::ConnectorType::OUTPUT}
	, outGyro{"outGyro", BVS::ConnectorType::OUTPUT}
	, outMag{"outMag", BVS::ConnectorType::OUTPUT}
	, outTemp{"outTemp", BVS::ConnectorType::OUTPUT}
	, outDUOFrame{"outDUOFrame", BVS::ConnectorType::OUTPUT}
	, duo_res{ 0, 0, 0, 0, 0, 0}
	, showDuoInfo{bvs.config.getValue<bool>(info.conf + ".showDuoInfo", false)}
	, showDuoParams{bvs.config.getValue<bool>(info.conf + ".showDuoParams", false)}
	, showDuoFOV{bvs.config.getValue<bool>(info.conf + ".showDuoFOV", false)}
	, showDuoStereo{bvs.config.getValue<bool>(info.conf + ".showDuoStereo", false)}
	, blockModule{bvs.config.getValue<bool>(info.conf + ".blockModule", true)}
	, width{bvs.config.getValue<int>(info.conf + ".width", 752)}
	, height{bvs.config.getValue<int>(info.conf + ".height", 480)}
	, binning{bvs.config.getValue<int>(info.conf + ".binning", -1)}
	, fps{bvs.config.getValue<float>(info.conf + ".fps", -1)}
	, undistort{bvs.config.getValue<bool>(info.conf + ".undistort", true)}
	, autoCorrect{bvs.config.getValue<bool>(info.conf + ".autoCorrect", false)}
	, autoQuantile{bvs.config.getValue<double>(info.conf + ".autoQuantile", 0.05)}
	, autoTargetMean{bvs.config.getValue<unsigned int>(info.conf + ".autoTargetMean", 96)}
	, autoAttenuation{bvs.config.getValue<double>(info.conf + ".autoAttenuation", 0.5)}
	, antiReadNoise{bvs.config.getValue<bool>(info.conf + ".antiReadNoise", false)}
	, noiseFrameFileLeft{bvs.config.getValue<std::string>(info.conf + ".noiseFrameFileLeft", {})}
	, noiseFrameFileRight{bvs.config.getValue<std::string>(info.conf + ".noiseFrameFileRight", {})}
	, noiseLeft{}
	, noiseRight{}
	, timeStamp(0)
{
	if (OpenDUO(&duo)) {
		LOG(1, "Connection to DUO established!");

		if (showDuoInfo) {
			char tmp[260];
			LOG(2, "DUO3D INFO:");
			LOG(2,                                           "Library Version:  v" << GetDUOLibVersion());
			printParam(GetDUODeviceName(duo, tmp), tmp,      "Device Name:      ");
			printParam(GetDUOSerialNumber(duo, tmp), tmp,    "Serial Number:    ");
			printParam(GetDUOFirmwareVersion(duo, tmp), tmp, "Firmware Version: v");
			printParam(GetDUOFirmwareBuild(duo, tmp), tmp,   "Firmware Build:   ");
		}

		if (EnumerateDUOResolutions(&duo_res, 1, width, height, binning, fps)==0) {
			DUOResolutionInfo res[100];
			int num = EnumerateDUOResolutions(res, 100, width, height, binning, fps);
			LOG(1, "Available resolutions for " << width << "x" << height << "@" << fps << " - mode: " << binning << ":");
			for (int i=0; i<=num; i++)
				LOG(0, res[i].width
						<< "x" << res[i].height
						<< "@" << res[i].fps
						<< "fps [minFPS:" << res[i].minFps
						<< "-maxFPS:" << res[i].maxFps
						<< "] - mode: " << res[i].binning);
		}

		if (SetDUOResolutionInfo(duo, duo_res)) {
		LOG(2, "Resolution Info:  " << duo_res.width << "x" << duo_res.height
				<< "@" << duo_res.fps << "fps - binning mode: " << duo_res.binning);
		}

		setParam(SetDUOExposure(duo, bvs.config.getValue(info.conf + ".exposure", 80.0f)), "Exposure");
		double exp = bvs.config.getValue(info.conf + ".exposureMS", 0.0f);
		if (exp>0.0f) setParam(SetDUOExposureMS(duo, exp), "Exposure in ms");
		setParam(SetDUOAutoExposure(duo, bvs.config.getValue(info.conf + ".autoExposure", false)), "AutoExposure");
		setParam(SetDUOGain(duo, bvs.config.getValue(info.conf + ".gain", 0)), "Gain");
		setParam(SetDUOHFlip(duo, bvs.config.getValue(info.conf + ".hFlip", false)), "Horizontal Flip");
		setParam(SetDUOVFlip(duo, bvs.config.getValue(info.conf + ".vFlip", false)), "Vertical Flip");
		setParam(SetDUOCameraSwap(duo, bvs.config.getValue(info.conf + ".cameraSwap", false)), "Camera Swap");
		setParam(SetDUOLedPWM(duo, bvs.config.getValue(info.conf + ".led", 0.1f)), "LED PWM");
		setParam(SetDUOUndistort(duo, undistort), "Undistort");

		if (undistort) {
			bool b;
			GetDUOCalibrationPresent(duo, &b);
			if (!b) LOG(0, "Undistort requested but no calibration available on DUO device!");
		}

		if (showDuoParams) {
			LOG(2, "DUO3D PARAMETERS:");
			double d;
			bool b;
			uint32_t w, h;
			printParam(GetDUOAutoExposure(duo, &b), b,       "AutoExposure:     ");
			printParam(GetDUOExposure(duo, &d), d,           "Exposure [0,100]: ");
			printParam(GetDUOExposureMS(duo, &d), d,         "Exposure [ms]:    ");
			printParam(GetDUOGain(duo, &d), d,               "Gain [0, 100]:    ");
			printParam(GetDUOHFlip(duo, &b), b,              "Horizontal Flip:  ");
			printParam(GetDUOVFlip(duo, &b), b,              "Vertical Flip:    ");
			printParam(GetDUOCameraSwap(duo, &b), b,         "Camera Swap:      ");
			printParam(GetDUOLedPWM(duo, &d), d,             "LED PWM [0,100]:  ");
			printParam(GetDUOCalibrationPresent(duo, &b), b, "Calibration:      ");
			printParam(GetDUOUndistort(duo, &b), b,          "Undistort:        ");
			if (GetDUOFrameDimension(duo, &w, &h)) LOG(2,    "Image Size [WxH]: " << w << "x" << h);
		}
		if (showDuoFOV) {
			LOG(2, "DUO3D FOV:");
			double d[4];
			GetDUOFOV(duo, d);
			LOG(2, "Distorted Left: H" << d[0] << " V"<< d[1] << " Right: H" << d[2] << " V" << d[3]);
			GetDUORectifiedFOV(duo, d);
			LOG(2, "Rectified Left: H" << d[0] << " V"<< d[1] << " Right: H" << d[2] << " V" << d[3]);
		}

		if (showDuoStereo) {
			LOG(2, "DUO3D STEREO:");
			DUO_STEREO st;
			GetDUOStereoParameters(duo, &st);

			std::function<void(const std::string&, const double[9], const double[8])> showIntrinsics = [&](const std::string& str, const double m[9], const double d[8]) {
				LOG(2, "INTRINSICS " << str << ": [M]" << std::fixed);
				LOG(2, "⎡ " << m[0] << " " << m[1] << " " << m[2] << "⎤" << "   Distortion Coefficients");
				LOG(2, "⎢ " << m[3] << " " << m[4] << " " << m[5] << "⎥" << "   K: " << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << " " << d[4] << " " << d[5]);
				LOG(2, "⎣ " << m[6] << " " << m[7] << " " << m[8] << "⎦" << "   P: " << d[6] << " " << d[7]);
			};
			showIntrinsics("LEFT CAMERA:", st.M1, st.D1);
			showIntrinsics("RIGHT CAMERA:", st.M2, st.D2);

			LOG(2, "EXTRINSICS: [R|T]");
			LOG(2, "⎡ " << st.R[0] << " " << st.R[1] << " " << st.R[2] << " | " << st.T[0] << "⎤");
			LOG(2, "⎢ " << st.R[3] << " " << st.R[4] << " " << st.R[5] << " | " << st.T[1] << "⎥");
			LOG(2, "⎣ " << st.R[6] << " " << st.R[7] << " " << st.R[8] << " | " << st.T[2] << "⎦");
		}
	} else {
		LOG(0, "Could not connect to DUO!");
	}

	if (StartDUO(duo, DUOCallback, nullptr)) {
		LOG(1, "Starting frame capture on DUO!");
	} else {
		LOG(0, "Could not start frame capture on DUO!");
	}

	if (antiReadNoise) {
		if (noiseFrameFileLeft.empty()) {
			LOG(0, "No left noise frame given!");
		} else {
			noiseLeft = cv::imread(noiseFrameFileLeft, cv::IMREAD_GRAYSCALE);
			if (noiseLeft.empty())
				LOG(0, "Could not load left noise frame!");
		}
		if (noiseFrameFileRight.empty()) {
			LOG(0, "No right noise frame given!");
		} else {
			noiseRight = cv::imread(noiseFrameFileRight, cv::IMREAD_GRAYSCALE);
			if (noiseRight.empty())
				LOG(0, "Could not load right noise frame!");
		}
	}
}



Duo3D::~Duo3D()
{
	StopDUO(duo);
	CloseDUO(duo);
	duo = nullptr;
}



BVS::Status Duo3D::execute()
{
	using namespace std::chrono;
	using namespace std::literals;
	high_resolution_clock::time_point start = high_resolution_clock::now();

	if (duo==nullptr) return BVS::Status::SHUTDOWN;

	if (duo_frame==nullptr) {
		LOG(2, "Waiting max 500ms for first duo frame!");
		for (int i=0; i<50; i++) {
			std::this_thread::sleep_for(10ms);
			if (duo_frame!=nullptr) break;
		}
	}
	if (duo_frame==nullptr) return BVS::Status::WAIT;

	std::lock_guard<std::mutex> lock{mutex};

	if (timeStamp==duo_frame->timeStamp)
		return BVS::Status::WAIT;
	timeStamp = duo_frame->timeStamp;

	if (outL.active()) {
		cv::Mat duo_left(cv::Size{(int)duo_frame->width, (int)duo_frame->height}, CV_8UC1);
		duo_left.data = (unsigned char*)duo_frame->leftData;
		if (antiReadNoise) {
			duo_left = duo_left - noiseLeft;
		}
		outL.send(duo_left);
	}
	if (outR.active()) {
		cv::Mat duo_right(cv::Size{(int)duo_frame->width, (int)duo_frame->height}, CV_8UC1);
		duo_right.data = (unsigned char*)duo_frame->rightData;
		if (antiReadNoise) {
			duo_right = duo_right - noiseRight;
		}
		outR.send(duo_right);
	}
	if (outTime.active()) outTime.send(duo_frame->timeStamp);
	if (outAccel.active()) outAccel.send({{duo_frame->IMUData->accelData[0], duo_frame->IMUData->accelData[1], duo_frame->IMUData->accelData[2]}});
	if (outGyro.active()) outGyro.send({{duo_frame->IMUData->gyroData[0], duo_frame->IMUData->gyroData[1], duo_frame->IMUData->gyroData[2]}});
	if (outTemp.active()) outTemp.send(duo_frame->IMUData->tempData);
	if (outDUOFrame.active()) outDUOFrame.send(*duo_frame);

	if (autoCorrect) autoCorrection();

	if (blockModule) {
		long duration{duration_cast<milliseconds>(high_resolution_clock::now()-start).count()};
		if (duration<1000/fps)
			std::this_thread::sleep_for(milliseconds{
				(long)(1000/fps-duration)});
	}

	return BVS::Status::OK;
}



void Duo3D::DUOCallback(const PDUOFrame pFrameData, void * pUserData)
{
	(void) pUserData;

	std::lock_guard<std::mutex> lock{mutex};
	duo_frame = pFrameData;
}



void Duo3D::setParam(bool success, std::string parameterName)
{
	if (!success)
		LOG(0, "Failed to set parameter: " << parameterName << "!");
}



void Duo3D::autoCorrection() {
	cv::Mat img;
	if (outL.active()) img = *outL;
	else if (outR.active()) img = *outR;
	if (!img.empty()) {
		// calculate quantiles
		std::array<uint32_t, 256> hist{ {0} };
		uint32_t hist_sum = 0;
		for (auto pix=img.begin<uchar>(); pix<img.end<uchar>(); pix++) hist[*pix]++;
		for (const auto& h: hist) hist_sum+= h;
		uint32_t quant_low = autoQuantile * hist_sum;
		uint32_t quant_high = (1.0 - autoQuantile) * hist_sum;

		// calculate mean (disregard quantiles)
		uint32_t acc = 0;
		uint32_t mean_sum = 0;
		uint32_t mean_elements = 0;
		size_t i{};
		// discard lower quantile
		for ( i=0; i<hist.size(); ++i) {
			acc += hist[i];
			if (acc>quant_low) break;
		}
		// partly add current histogram bin
		mean_sum = i*(acc-quant_low);
		mean_elements = acc-quant_low;
		// collect data for mean until upper quantile reached
		for ( ++i; i<hist.size(); ++i) {
			acc += hist[i];
			mean_sum += i*hist[i];
			mean_elements += hist[i];
			if (acc>quant_high) break;
		}
		// partly add current histogram bin
		mean_sum += i*(acc-quant_high);
		mean_elements += acc-quant_high;
		// calculate mean
		double mean = mean_sum / double(mean_elements);

		// calculate new gain, add 0.1 to prevent 0 stall, cut to 2 digits
		double gain{};
		GetDUOGain(duo, &gain);
		gain = (gain+0.1)*(autoAttenuation*(autoTargetMean/mean-1.0)+1.0);
		gain = int(gain*100)/100.0;
		SetDUOGain(duo, gain);
	}
}



// UNUSED
BVS::Status Duo3D::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(Duo3D)

