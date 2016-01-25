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
	, blockModule{bvs.config.getValue<bool>(info.conf + ".blockModule", true)}
	, width{bvs.config.getValue<int>(info.conf + ".width", 752)}
	, height{bvs.config.getValue<int>(info.conf + ".height", 480)}
	, binning{bvs.config.getValue<int>(info.conf + ".binning", -1)}
	, fps{bvs.config.getValue<float>(info.conf + ".fps", -1)}
	, autoCorrect{bvs.config.getValue<bool>(info.conf + ".autoCorrect", false)}
	, autoQuantile{bvs.config.getValue<double>(info.conf + ".autoQuantile", 0.05)}
	, autoTargetMean{bvs.config.getValue<unsigned int>(info.conf + ".autoTargetMean", 96)}
	, autoAttenuation{bvs.config.getValue<double>(info.conf + ".autoAttenuation", 0.8)}
	, timeStamp(0)
{
	if (OpenDUO(&duo)) {
		LOG(1, "Connection to DUO established!");

		if (showDuoInfo) {
			char tmp[260];
			LOG(2, "DUO3D INFO:");
			LOG(2,                                           "Library Version:  v" << GetLibVersion());
			printParam(GetDUODeviceName(duo, tmp), tmp,      "Device Name:      ");
			printParam(GetDUOSerialNumber(duo, tmp), tmp,    "Serial Number:    ");
			printParam(GetDUOFirmwareVersion(duo, tmp), tmp, "Firmware Version: v");
			printParam(GetDUOFirmwareBuild(duo, tmp), tmp,   "Firmware Build:   ");
		}

		if (EnumerateResolutions(&duo_res, 1, width, height, binning, fps)==0) {
			DUOResolutionInfo res[100];
			int num = EnumerateResolutions(res, 100, width, height, binning, fps);
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
		setParam(SetDUOGain(duo, bvs.config.getValue(info.conf + ".gain", 0)), "Gain");
		setParam(SetDUOHFlip(duo, bvs.config.getValue(info.conf + ".hFlip", false)), "Horizontal Flip");
		setParam(SetDUOVFlip(duo, bvs.config.getValue(info.conf + ".vFlip", false)), "Vertical Flip");
		setParam(SetDUOCameraSwap(duo, bvs.config.getValue(info.conf + ".cameraSwap", false)), "Camera Swap");
		setParam(SetDUOLedPWM(duo, bvs.config.getValue(info.conf + ".led", 0.0f)), "LED PWM");

		if(showDuoParams) {
			LOG(2, "DUO3D PARAMETERS:");
			double d;
			bool b;
			uint32_t w, h;
			printParam(GetDUOExposure(duo, &d), d,           "Exposure [0,100]: ");
			printParam(GetDUOExposureMS(duo, &d), d,         "Exposure [ms]:    ");
			printParam(GetDUOGain(duo, &d), d,               "Gain [0, 100]:    ");
			printParam(GetDUOHFlip(duo, &b), b,              "Horizontal Flip:  ");
			printParam(GetDUOVFlip(duo, &b), b,              "Vertical Flip:    ");
			printParam(GetDUOCameraSwap(duo, &b), b,         "Camera Swap:      ");
			printParam(GetDUOLedPWM(duo, &d), d,             "LED PWM [0,100]:  ");
			if (GetDUOFrameDimension(duo, &w, &h)) LOG(2,    "Image Size [WxH]: " << w << "x" << h);
		}
	} else {
		LOG(0, "Could not connect to DUO!");
	}

	if (StartDUO(duo, DUOCallback, nullptr)) {
		LOG(1, "Starting frame capture on DUO!");
	} else {
		LOG(0, "Could not start frame capture on DUO!");
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
		outL.send(duo_left);
	}
	if (outR.active()) {
		cv::Mat duo_right(cv::Size{(int)duo_frame->width, (int)duo_frame->height}, CV_8UC1);
		duo_right.data = (unsigned char*)duo_frame->rightData;
		outR.send(duo_right);
	}
	if (outTime.active()) outTime.send(duo_frame->timeStamp);
	if (outAccel.active()) outAccel.send({{duo_frame->accelData[0], duo_frame->accelData[1], duo_frame->accelData[2]}});
	if (outGyro.active()) outGyro.send({{duo_frame->gyroData[0], duo_frame->gyroData[1], duo_frame->gyroData[2]}});
	if (outMag.active()) outMag.send({{duo_frame->magData[0], duo_frame->magData[1], duo_frame->magData[2]}});
	if (outTemp.active()) outTemp.send(duo_frame->tempData);
	if (outDUOFrame.active()) outDUOFrame.send(*duo_frame);

	if (autoCorrect) autoCorrection();

	if (blockModule) {
		long duration{duration_cast<milliseconds>(start-high_resolution_clock::now()).count()};
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
	//printf("DUO Callback captured a frame, timestamp: %d",duo_frame->timeStamp);
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
		uint32_t hist_q05 = autoQuantile / 100.0 * hist_sum;
		uint32_t hist_q95 = (100 - autoQuantile) / 100.0 * hist_sum;

		// calculate mean (disregard quantiles)
		uint32_t acc = 0;
		uint32_t mean_sum = 0;
		uint32_t mean_elements = 0;
		for (size_t i=0; i<hist.size(); i++) {
			acc += hist[i];
			if (acc<hist_q05) continue;
			if (acc>hist_q95) break;
			mean_sum += i*hist[i];
			mean_elements += hist[i];
		}
		uint32_t mean = mean_sum / double(mean_elements);
		// TODO: check for mean == 0!!!
		
		// calculate new gain
		double gain = 0;
		GetDUOGain(duo, &gain);
		gain = (autoAttenuation * (autoTargetMean - mean) / double(mean) + 1.0 ) * gain;
		if (gain==0) gain = 0.1;
		SetDUOGain(duo, gain);

		// TODO: consider exposure time (exp < 1000/fps)
		// TODO denoise gain dependent!!!
	}
}



// UNUSED
BVS::Status Duo3D::debugDisplay()
{
	return BVS::Status::OK;
}



/** This calls a macro to create needed module utilities. */
BVS_MODULE_UTILITIES(Duo3D)

