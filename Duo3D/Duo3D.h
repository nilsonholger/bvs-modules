#ifndef DUO3D_H
#define DUO3D_H

#include <functional>

#include "bvs/module.h"
#include "DUOLib.h"
#include "opencv2/opencv.hpp"



/** This is the Duo3D module.
 * Please add sufficient documentation to enable others to use it.
 * Include information about: Dependencies, Inputs, Outputs, Configuration Options...
 */
class Duo3D : public BVS::Module
{
	public:
		/** Your module constructor.
		 * Please do not change the signature, as it will be called by the
		 * framework.
		 * You can use the constructor/destructor pair to create/destroy your data.
		 * @param[in] info Your modules information, will be set by framework.
		 * @param[in] bvs Reference to framework info for e.g. config option retrieval.
		 */
		Duo3D(BVS::ModuleInfo info, const BVS::Info& bvs);

		/** Your module destructor. */
		~Duo3D();

		/** Execute function doing all the work.
		 * This function is executed exactly once during each started
		 * round/step of the framework. It is supposed to contain the actual
		 * work of your module.
		 */
		BVS::Status execute();

		/** UNUSED
		 * @return Module's status.
		 */
		BVS::Status debugDisplay();

	private:
		const BVS::ModuleInfo info; /**< Your module metadata, set by framework. */
		BVS::Logger logger; /**< Your logger instance. @see Logger */
		const BVS::Info& bvs; /**< Your Info reference. @see Info */

		/** Example Connector used to retrieve/send data from/to other modules.
		 * @see Connector
		 */
		//BVS::Connector<int> input;
		BVS::Connector<cv::Mat> outL;
		BVS::Connector<cv::Mat> outR;

		DUOInstance duo = NULL;
		DUOResolutionInfo duo_res;
		static PDUOFrame duo_frame;
		static void DUOCallback(const PDUOFrame pFrameData, void* pUserData);

		bool showDuoInfo;
		bool showDuoParams;

		int width;
		int height;
		int binning;
		float fps;

		void setParam(bool success, std::string parameterName);
		template<typename T> void printParam(bool success, T& t, std::string message);

		Duo3D(const Duo3D&) = delete; /**< -Weffc++ */
		Duo3D& operator=(const Duo3D&) = delete; /**< -Weffc++ */
};



template<typename T> void Duo3D::printParam(bool success, T& t, std::string message)
{
	if (success) {
		LOG(1, message << t);
	} else {
		LOG(0, "Could not set: " << message.erase(message.find_first_of(':'), std::string::npos));
	}
}



#endif //DUO3D_H

