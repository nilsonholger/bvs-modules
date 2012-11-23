#include "StereoCVCUDA.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to its config.
// However, you should use it to create your data structures etc.
StereoCVCUDA::StereoCVCUDA(BVS::ModuleInfo info, const BVS::Info& bvs)
	: BVS::Module(),
	info(info),
	logger(info.id),
	config("StereoCVCUDA", 0, nullptr), // "StereoCVCUDAConfig.txt"),
	// at this point config has already loaded 'StereoCVCUDAConfig.txt", so
	// you can use config to retrieve settings in the initialization list, e.g.
	// yourSwitch(config.getValue<bool>(info.conf + ".yourSwitch, false));
	bvs(bvs),
	input0("input0", BVS::ConnectorType::INPUT),
	input1("input1", BVS::ConnectorType::INPUT),
	depthImage("depthImage", BVS::ConnectorType::OUTPUT),
	in0(),
	in1(),
	grey0(),
	grey1(),
	gpuMat0(),
	gpuMat1(),
	disparity(),
	switchInputs(false),
	stereoAlgo(0),
	bmGPU(),
	bpGPU(),
	csGPU(),
	estimate(true)
{
	bmGPU.preset = 0;
	bmGPU.ndisp = 64;
	bmGPU.winSize = 19;
	bmGPU.avergeTexThreshold = 2;

	bpGPU.ndisp = 64;
	bpGPU.iters = 5;
	bpGPU.levels = 5;
	bpGPU.msg_type = CV_32F;

	csGPU.ndisp = 128;
	csGPU.iters = 8;
	csGPU.levels = 4;
	csGPU.nr_plane = 4;
}



// This is your module's destructor.
// See the constructor for more info.
StereoCVCUDA::~StereoCVCUDA()
{

}



// Put all your work here.
BVS::Status StereoCVCUDA::execute()
{
	if (switchInputs)
	{
		if (!input0.receive(in1) || !input1.receive(in0)) return BVS::Status::NOINPUT;
	}
	else
	{
		if (!input0.receive(in0) || !input1.receive(in1)) return BVS::Status::NOINPUT;
	}

	if (in0.empty() || in1.empty()) return BVS::Status::NOINPUT;

	if (in0.type()!=CV_8UC1 || in1.type()!=CV_8UC1)
	{
		//TODO test cv::gpu::cvtColor...
		cv::cvtColor(in0, grey0, CV_RGB2GRAY);
		cv::cvtColor(in1, grey1, CV_RGB2GRAY);
	}
	in0 = grey0;
	in1 = grey1;

	cv::pyrDown(in0, grey0);
	cv::pyrDown(in1, grey1);

	gpuMat0.upload(grey0);
	gpuMat1.upload(grey1);

	if (estimate)
	{
	bpGPU.estimateRecommendedParams(in0.cols, in0.rows, bpGPU.ndisp, bpGPU.iters, bpGPU.levels);
	csGPU.estimateRecommendedParams(in0.cols, in0.rows, csGPU.ndisp, csGPU.iters, csGPU.levels, csGPU.nr_plane);
		estimate = false;
	}

	switch (stereoAlgo)
	{
		case 0: bmGPU(gpuMat0, gpuMat1, disparity); break;
		case 1: bpGPU(gpuMat0, gpuMat1, disparity); break;
		case 2: csGPU(gpuMat0, gpuMat1, disparity); break;
	}

	cv::Mat grey;
	disparity.download(grey);
	//TODO needed? grey.convertTo(grey, CV_8U, 1./16.);

	cv::Mat out;
	cv::gpu::GpuMat color;
	switch (stereoAlgo)
	{
		case 0: cv::gpu::drawColorDisp(disparity, color, bmGPU.ndisp); break;
		case 1: cv::gpu::drawColorDisp(disparity, color, bpGPU.ndisp); break;
		case 2: cv::gpu::drawColorDisp(disparity, color, csGPU.ndisp); break;
	}
	color.download(out);
	cv::putText(out, bvs.getFPS(), cv::Point(10, 30),
			CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(0, 0, 255), 2);

	//cv::Mat edges;
	//cv::gpu::GpuMat canny;
	//cv::gpu::GaussianBlur(disparity, canny, cv::Size(31,31), 0);
	//cv::gpu::Canny(canny, disparity, 1.0, 2.0);
	//disparity.download(edges);

	cv::imshow("grey", grey);
	cv::imshow("out", out);
	//cv::imshow("0", grey0);
	//cv::imshow("1", grey1);
	//cv::imshow("edges", edges);

	handleInput(cv::waitKey(1));
	return BVS::Status::OK;
}



void StereoCVCUDA::handleInput(char c)
{
	switch (c)
	{
		case 'h':
			LOG(1, "\n\
usage:\n\
    x   - switch inputs: " << switchInputs << "\n\
    a   - change algorithm: " << (stereoAlgo==0? "BM": stereoAlgo==1? "BP": "CSBP") << "\n\
    e   - estimate BP and CSBP settings\n\
BM:\n\
    p   - prefilter on/off: " << bmGPU.preset << "\n\
    n/N - change number of disparities: " << bmGPU.ndisp << "\n\
    w/W - change window size: " << bmGPU.winSize << "\n\
    t/T - change texture averaging threshold: " << bmGPU.avergeTexThreshold << "\n\
BP:\n\
    n/N - change number of disparities: " << bpGPU.ndisp << "\n\
    i/I - change number of iterations " << bpGPU.iters << "\n\
    l/L - change number of levels " << bpGPU.levels << "\n\
CS:\n\
    n/N - change number of disparities: " << csGPU.ndisp << "\n\
    i/I - change number of iterations " << csGPU.iters << "\n\
    l/L - change number of levels " << csGPU.levels << "\n\
"); break;

		case 'x': switchInputs = !switchInputs; break;
		case 'a':
			stereoAlgo = (stereoAlgo + 1) % 3;
			LOG(1, "algo: " << (stereoAlgo==0? "BM": stereoAlgo==1? "BP": "CSBP"));
			break;
		case 'e': estimate = true; break;
		case 'p':
			bmGPU.preset = (bmGPU.preset + 1) % 2;
			LOG(1, "preset: " << bmGPU.preset);
			break;
		case 'N':
			switch (stereoAlgo)
			{
				case 0: bmGPU.ndisp += 8; LOG(1, "ndisp: " << bmGPU.ndisp); break;
				case 1: bpGPU.ndisp += 8; LOG(1, "ndisp: " << bpGPU.ndisp); break;
				case 2: csGPU.ndisp += 8; LOG(1, "ndisp: " << csGPU.ndisp); break;
			}
			break;
		case 'n':
			switch (stereoAlgo)
			{
				case 0: bmGPU.ndisp = bmGPU.ndisp-8 < 8 ? 8: bmGPU.ndisp-8; LOG(1, "ndisp: " << bmGPU.ndisp); break;
				case 1: bpGPU.ndisp = bpGPU.ndisp-8 < 8 ? 8: bpGPU.ndisp-8; LOG(1, "ndisp: " << bpGPU.ndisp); break;
				case 2: csGPU.ndisp = csGPU.ndisp-8 < 8 ? 8: csGPU.ndisp-8; LOG(1, "ndisp: " << csGPU.ndisp); break;
			}
			break;
		case'W':
			bmGPU.winSize += 1;
			LOG(1, "winSize: " << bmGPU.winSize);
			break;
		case'w':
			bmGPU.winSize -= 1;
			bmGPU.winSize < 2 ? bmGPU.winSize = 2: bmGPU.winSize;
			LOG(1, "winSize: " << bmGPU.winSize);
			break;
		case'T':
			bmGPU.avergeTexThreshold += 1;
			LOG(1, "tex: " << bmGPU.avergeTexThreshold);
			break;
		case't':
			bmGPU.avergeTexThreshold -= 1;
			bmGPU.avergeTexThreshold < 0 ? bmGPU.avergeTexThreshold = 0: bmGPU.avergeTexThreshold;
			LOG(1, "tex: " << bmGPU.avergeTexThreshold);
			break;
		case 'I':
			switch (stereoAlgo)
			{
				case 1: bpGPU.iters += 1; LOG(1, "iters: " << bpGPU.iters); break;
				case 2: csGPU.iters += 1; LOG(1, "iters: " << csGPU.iters); break;
			}
			break;
		case 'i':
			switch (stereoAlgo)
			{
				case 1: bpGPU.iters = bpGPU.iters-1 < 1 ? 1: bpGPU.iters-1; LOG(1, "iters: " << bpGPU.iters); break;
				case 2: csGPU.iters = csGPU.iters-1 < 1 ? 1: csGPU.iters-1; LOG(1, "iters: " << csGPU.iters); break;
			}
			break;
		case 'L':
			switch (stereoAlgo)
			{
				case 1: bpGPU.levels += 1; LOG(1, "levels: " << bpGPU.levels); break;
				case 2: csGPU.levels += 1; LOG(1, "levels: " << csGPU.levels); break;
			}
			break;
		case 'l':
			switch (stereoAlgo)
			{
				case 1: bpGPU.levels = bpGPU.levels-1 < 1 ? 1: bpGPU.levels-1; LOG(1, "levels: " << bpGPU.levels); break;
				case 2: csGPU.levels = csGPU.levels-1 < 1 ? 1: csGPU.levels-1; LOG(1, "levels: " << csGPU.levels); break;
			}
			break;
		case 27: exit (0); break;
	}
}



// UNUSED
BVS::Status StereoCVCUDA::debugDisplay()
{
	return BVS::Status::OK;
}

