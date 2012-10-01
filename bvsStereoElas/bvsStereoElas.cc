#include "bvsStereoElas.h"



bvsStereoElas::bvsStereoElas(const std::string id, const BVS::Info& bvs)
	: BVS::Module(),
	id(id),
	logger(id),
	config("bvsStereoElas", 0, nullptr, "bvsStereoElasConfig.txt"),
	bvs(bvs),
	inL("inL", BVS::ConnectorType::INPUT),
	inR("inR", BVS::ConnectorType::INPUT),
	outL("outL", BVS::ConnectorType::OUTPUT),
	outR("outR", BVS::ConnectorType::OUTPUT),
	discardTopLines(config.getValue<int>(id+".discardTopLines", 0)),
	discardBottomLines(config.getValue<int>(id+".discardBottomLines", 0)),
	scalingFactor(config.getValue<float>(id+".scalingFactor", 1)),
	sliceCount(config.getValue<int>(id+".sliceCount", 1)),
	sliceOverlap(config.getValue<int>(id+".sliceOverlap", 10)),
	sliceExit(false),
	runningThreads(0),
	masterMutex(),
	sliceMutex(),
	masterLock(masterMutex),
	monitor(),
	threadMonitor(),
	threads(),
	flags(sliceCount),
	tmpL(),
	tmpR(),
	left(),
	right(),
	dispL(),
	dispR(),
	dimensions(),
	param(),
	elas(param)
{
	if (sliceCount<=0)
	{
		LOG(0, "ERROR: sliceCount <= 0! Aborting...");
		exit(1);
	}

	param.postprocess_only_left = false;
	elas = Elas(param);

	if (sliceCount!=1)
	{
		runningThreads.store(0, std::memory_order_release);
		for (int i=0; i<sliceCount; i++)
			threads.push_back(std::thread(&bvsStereoElas::sliceThread, this, i));
		flags.resize(sliceCount);
		for (auto f: flags) f = false;
	}
}



bvsStereoElas::~bvsStereoElas()
{
	if (sliceCount!=1)
	{
		sliceExit = true;
		for (auto f: flags) f = true;
		runningThreads.store(sliceCount);
		threadMonitor.notify_all();
		for (auto& t: threads) if (t.joinable()) t.join();
	}
}



BVS::Status bvsStereoElas::execute()
{
	if (!inL.receive(tmpL) || !inR.receive(tmpR)) return BVS::Status::NOINPUT;
	if (tmpL.empty() || tmpR.empty()) return BVS::Status::NOINPUT;

	cv::resize(tmpL, left, cv::Size(tmpL.cols/scalingFactor, tmpL.rows/scalingFactor), 0, 0, cv::INTER_AREA);
	cv::resize(tmpR, right, cv::Size(tmpR.cols/scalingFactor, tmpR.rows/scalingFactor), 0, 0, cv::INTER_AREA);

	if (dispL.size()==cv::Size())
	{
		dispL = cv::Mat(left.size(), CV_32FC1);
		dispR = cv::Mat(left.size(), CV_32FC1);
		dimensions[0] = left.cols;
		dimensions[1] = (left.rows-discardTopLines-discardBottomLines)/sliceCount; //TODO + sliceOverlap;
		dimensions[2] = dimensions[0];
	}

	if (sliceCount!=1)
	{
		for (auto f: flags) f = true;
		runningThreads.store(sliceCount);
		threadMonitor.notify_all();
		monitor.wait(masterLock, [&](){ return runningThreads.load()==0; });
	}
	else
	{
		elas.process(left.data, right.data, (float*)dispL.data, (float*)dispR.data, dimensions);
	}

	outL.send(dispL);
	outR.send(dispR);

	//float disp_max = 0;
	//for (int32_t i=0; i<left.cols*left.rows; i++) {
		//if (*((float*)dispL.data+i)>disp_max) disp_max = *((float*)dispL.data+i);
		//if (*((float*)dispR.data+i)>disp_max) disp_max = *((float*)dispR.data+i);
	//}

	//cv::Mat showL = cv::Mat(left.size(), CV_8UC1);
	//cv::Mat showR = cv::Mat(left.size(), CV_8UC1);
	//for (int32_t i=0; i<left.cols*left.rows; i++) {
		//*(showL.data+i) = (uint8_t)std::max(255.0* *((float*)dispL.data+i)/disp_max,0.0);
		//*(showR.data+i) = (uint8_t)std::max(255.0* *((float*)dispR.data+i)/disp_max,0.0);
	//}

	//LOG(3, "fps: " << bvs.getFPS());
	//cv::putText(showL, bvs.getFPS(), cv::Point(10, 30),
			//CV_FONT_HERSHEY_SIMPLEX, 1.0f, cvScalar(255, 255, 255), 2);
	//cv::imshow("iL", left);
	//cv::imshow("iR", right);
	//cv::imshow("dL", showL);
	//cv::imshow("dR", showR);
	//cv::waitKey(1);

	return BVS::Status::OK;
}



void bvsStereoElas::sliceThread(int id)
{
	BVS::nameThisThread("elas.slice");
	std::unique_lock<std::mutex> lock(sliceMutex);

	while (!sliceExit)
	{
		threadMonitor.wait(lock, [&](){ return flags[id]; });

		int offset = (discardTopLines+id*dimensions[1]) * dimensions[0]; //TODO - sliceOverlap; //except for id==0
		elas.process(left.data+offset, right.data+offset, (float*)dispL.data+offset, (float*)dispR.data+offset, dimensions);

		runningThreads.fetch_sub(1);
		flags[id] = false;
		monitor.notify_one();
	}
	monitor.notify_one();
}



BVS::Status bvsStereoElas::debugDisplay()
{
	return BVS::Status::OK;
}



extern "C" {
	int bvsRegisterModule(std::string id, BVS::Info& bvs)
	{
		registerModule(id, new bvsStereoElas(id, bvs));

		return 0;
	}
}

