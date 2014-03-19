////////////////////////////////////////////////////////////////////////
// mjpeg GUI for browsers
//
// Manuel Martinez (manuel.martinez@kit.edu)
//
// license: LGPLv3

#include "uhttp.hpp"
//#include <uJpeg.hpp>
#include <turbojpeg.h>
#include <opencv/cv.h>

class MJPEG : public UHTTP {

	int q;
	std::map<std::string, uint64_t> timestamps;
	std::map<std::string, std::shared_future<std::string>> jpegs;
	
	static std::string getJPEG( cv::Mat3b img, int quality ) {
		
		tjhandle jpegHandle = tjInitCompress();
		uint8_t *jpegBuff = NULL;
		long unsigned int jpegSz = 0;

		tjCompress2(jpegHandle, &img(0,0)[0], img.cols, 0, img.rows, TJPF_BGR, &jpegBuff, &jpegSz, TJSAMP_420, quality, TJFLAG_FASTUPSAMPLE );
		std::string r((const char *)jpegBuff, size_t(jpegSz));

		tjFree(jpegBuff);
		tjDestroy(jpegHandle);
		return r;	
	}
	
public:

	MJPEG(int port = 80, int q = 75, int nworkers = 16) : UHTTP(port, nworkers), q(q) {
		
		(*this)("mjpeg", [&](boost::asio::ip::tcp::iostream &net, std::string , std::string url , std::string ) {
		
			std::replace(url.begin(), url.end(),'/',' ');
			std::istringstream iss(url);
			std::string command, name;
			iss >> command >> name;
		
			net << "HTTP/1.1 200 OK\r\n";
			net << "Content-Type: multipart/x-mixed-replace; boundary=myboundary\r\n\r\n";
			uint64_t old = 0;
			while (net << "--myboundary\r\n") {
				
				while (old==timestamps[name]) std::this_thread::sleep_for( std::chrono::milliseconds(5) );
				std::shared_future<std::string> j;
				{ Lock l(mtx); old = timestamps[name]; j = jpegs[name]; }
				
				std::string msg = j.get();
				net << "Content-Length: " << msg.size() << "\r\n";
				net << "Content-Type: image/jpeg\r\n\r\n";
				net << msg << "\r\n\r\n";
				net << std::flush;
			}
		});
		
		(*this)("", [](boost::asio::ip::tcp::iostream &net, std::string , std::string , std::string ) {
		
			net << "HTTP/1.1 200 OK\r\n";
			net << "Content-Type: text/html\r\n\r\n";
			net << "<html><body style='background:#000000 no-repeat center center url(/mjpeg/);background-size:contain;'></body></html>";
			net << "\r\n\r\n";
		});
	}

	void add( cv::Mat3b img, std::string name = "" ) {
		
		img = img.clone();
		Lock l(mtx);
		timestamps[name] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//		jpegs[name] = std::shared_future<std::string>(std::async(std::launch::deferred, [=](){ return uJpge::encode(img.data, img.cols, img.rows, q); }));
		jpegs[name] = std::shared_future<std::string>(std::async(std::launch::deferred, [=](){ return getJPEG(img, q); }));
	};
};

