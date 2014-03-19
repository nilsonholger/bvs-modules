////////////////////////////////////////////////////////////////////////
// micro HTTP server
//
// Manuel Martinez (manuel.martinez@kit.edu)
//
// license: LGPLv3

#include <thread>
#include <future>
#include <mutex>

#include <map>

#include <boost/asio.hpp>

class UHTTP {
public:
	typedef std::function<void(boost::asio::ip::tcp::iostream &, std::string, std::string, std::string)> CB;

protected:

	typedef std::lock_guard<std::mutex> Lock;
	std::mutex mtx;
	
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::endpoint endpoint;
	boost::asio::ip::tcp::acceptor acceptor;
	
	std::map<std::string,CB> cb;
	std::vector<std::future<void>> f;

	void start() {
		
		std::shared_ptr<boost::asio::ip::tcp::iostream> pNetStream(new boost::asio::ip::tcp::iostream);
		
		acceptor.async_accept(*pNetStream->rdbuf(), [=](const boost::system::error_code &error){ 

			boost::asio::ip::tcp::iostream &net = *pNetStream;

			if (error) return start();

			std::string get, url, hostLabel, host;
			if (not (net >>  get >> url)) return start();
			if (url[0]!='/') return start();
			if (get!="GET" and get!="POST") return start();
			while ((net >> host) and host != "Host:");
			if (host!="Host:") return start();
			if (not (net >> host)) return start();
			
			std::string command = "";
			if (url.find('/',1) != std::string::npos)
				command = url.substr(1, url.find('/',1)-1);

			CB c; {	Lock l(mtx); c = cb[command]; }
			if (c) c(net, get, url, host);
				
			start();
		} );
	}

public:

	UHTTP(int port = 80, int nworkers = 16) : 
		endpoint(boost::asio::ip::tcp::v4(), port), 
		acceptor(io_service, endpoint) {
			for (int i=0; i<nworkers; i++) 
				f.push_back(std::async(std::launch::async, [&](){start(); io_service.run();}));
		}
		
	~UHTTP() { io_service.stop(); }
	
	void operator()(std::string command, CB callback = CB()) { Lock l(mtx); cb[command] = callback; }
};

