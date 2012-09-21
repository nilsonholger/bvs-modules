#include "bvsStereoElas.h"



bvsStereoElas::bvsStereoElas(const std::string id, const BVS::Info& bvs)
	: BVS::Module(),
	id(id),
	logger(id),
	config("bvsStereoElas", 0, nullptr), // "bvsStereoElasConfig.txt"),
	bvs(bvs),
	inL("inL", BVS::ConnectorType::INPUT),
	inR("inR", BVS::ConnectorType::INPUT),
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
	param.postprocess_only_left = false;
	elas = Elas(param);
}



bvsStereoElas::~bvsStereoElas()
{

}



BVS::Status bvsStereoElas::execute()
{
	if (!inL.receive(tmpL) || !inR.receive(tmpR)) return BVS::Status::NOINPUT;

	cv::pyrDown(tmpL, left);
	cv::pyrDown(tmpR, right);

	if (dispL.size()==cv::Size())
	{
		dispL = cv::Mat(left.size(), CV_32FC1);
		dispR = cv::Mat(left.size(), CV_32FC1);
		dimensions[0] = left.cols;
		dimensions[1] = left.rows;
		dimensions[2] = left.cols;
	}

	elas.process(left.data,right.data,(float*)dispL.data,(float*)dispR.data,dimensions);

	float disp_max = 0;
	for (int32_t i=0; i<left.cols*left.rows; i++) {
		if (*((float*)dispL.data+i)>disp_max) disp_max = *((float*)dispL.data+i);
		if (*((float*)dispR.data+i)>disp_max) disp_max = *((float*)dispR.data+i);
	}

	cv::Mat showL = cv::Mat(left.size(), CV_8UC1);
	cv::Mat showR = cv::Mat(left.size(), CV_8UC1);
	for (int32_t i=0; i<left.cols*left.rows; i++) {
		*(showL.data+i) = (uint8_t)std::max(255.0* *((float*)dispL.data+i)/disp_max,0.0);
		*(showR.data+i) = (uint8_t)std::max(255.0* *((float*)dispR.data+i)/disp_max,0.0);
	}

	cv::imshow("iL", left);
	cv::imshow("iR", right);
	cv::imshow("dL", showL);
	cv::imshow("dR", showR);
	cv::waitKey(1);

	return BVS::Status::OK;
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

