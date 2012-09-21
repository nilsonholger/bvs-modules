#include "bvsStereoElas.h"
#include "elas/elas.h"



// This is your module's constructor.
// Please do not change its signature as it is called by the framework (so the
// framework actually creates your module) and the framework assigns the unique
// identifier and gives you access to its config.
// However, you should use it to create your data structures etc.
bvsStereoElas::bvsStereoElas(const std::string id, const BVS::Info& bvs)
	: BVS::Module(),
	id(id),
	logger(id),
	config("bvsStereoElas", 0, nullptr), // "bvsStereoElasConfig.txt"),
	// if you add BVSExampleConfig.txt to the config constructior, it will be
	// loaded immediately, so you can use config to retrieve settings in the
	// initialization list, e.g.
	// yourSwitch(config.getValue<bool>(id + ".yourSwitch, false));
	bvs(bvs),
	inL("inL", BVS::ConnectorType::INPUT),
	inR("inR", BVS::ConnectorType::INPUT),
	left(),
	right()
{

}



// This is your module's destructor.
// See the constructor for more info.
bvsStereoElas::~bvsStereoElas()
{

}



// Put all your work here.
BVS::Status bvsStereoElas::execute()
{
	if (!inL.receive(left) || !inR.receive(right)) return BVS::Status::NOINPUT;

	cv::Mat tmp;
	cv::Mat tmp2;
	cv::pyrDown(left, tmp);
	left = tmp;
	cv::pyrDown(right, tmp2);
	right = tmp2;
	cv::Mat dL(tmp);
	cv::Mat dR(tmp2);

	cv::imshow("iL", left);
	cv::imshow("iR", right);

	int32_t width  = left.cols;
	int32_t height = left.rows;
	const int32_t dims[3] = {width,height,width};
	float* D1_data = (float*)malloc(width*height*sizeof(float));
	float* D2_data = (float*)malloc(width*height*sizeof(float));

	Elas::parameters param;
	param.postprocess_only_left = false;
	Elas elas(param);
	elas.process(left.data,right.data,D1_data,D2_data,dims);

	float disp_max = 0;
	for (int32_t i=0; i<width*height; i++) {
		if (D1_data[i]>disp_max) disp_max = D1_data[i];
		if (D2_data[i]>disp_max) disp_max = D2_data[i];
	}

	for (int32_t i=0; i<width*height; i++) {
		*(dL.data+i) = (uint8_t)std::max(255.0*D1_data[i]/disp_max,0.0);
		*(dR.data+i) = (uint8_t)std::max(255.0*D2_data[i]/disp_max,0.0);
	}

	cv::imshow("dL", dL);
	cv::imshow("dR", dR);
	cv::waitKey(1);

	free(D1_data);
	free(D2_data);

	return BVS::Status::OK;
}



// UNUSED
BVS::Status bvsStereoElas::debugDisplay()
{
	return BVS::Status::OK;
}



// This function is called by the framework upon creating a module instance of
// this class. It creates the module and registers it within the framework.
// DO NOT CHANGE OR DELETE
extern "C" {
	int bvsRegisterModule(std::string id, BVS::Info& bvs)
	{
		registerModule(id, new bvsStereoElas(id, bvs));

		return 0;
	}
}

