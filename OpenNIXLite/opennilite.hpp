#pragma once

#include <opencv/cv.h>
#include <OpenNI2/OpenNI.h>
#include <functional>
#include <iostream>


class OpenNILite {
	
	template<typename T>
	struct Listener : public openni::VideoStream::NewFrameListener { 
		
		std::function<void(T)> cb;
		virtual void onNewFrame(openni::VideoStream &stream) {
			
			openni::VideoFrameRef frame;
			stream.readFrame(&frame);
			
			T img(frame.getHeight(), frame.getWidth());
			for (int y = 0; y < img.rows; y++)
				memcpy(img.ptr(y),((uint8_t *)frame.getData()) + y*frame.getStrideInBytes(), img.cols*img.elemSize());
			
			if (cb) cb(img);
		}
	};
	
	Listener<cv::Mat3b> colorListener;
	Listener<cv::Mat1s> depthListener;
	openni::Device device;
	openni::VideoStream depth, color;
public:
	
	OpenNILite() {
		openni::OpenNI::initialize();
		device.open(openni::ANY_DEVICE);
		device.setDepthColorSyncEnabled(true);
		device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );

		color.create(device, openni::SENSOR_COLOR);
		//for (auto &vm : color.getSensorInfo().getSupportedVideoModes()) std::cerr << "Color supports" << vm.getFps() << " " << vm.getPixelFormat() << " " << vm.getResolutionX() << " " << vm.getResolutionY() << std::endl;
		
		depth.create(device, openni::SENSOR_DEPTH);
		
		color.addNewFrameListener(&colorListener);
		depth.addNewFrameListener(&depthListener);
		
		color.start();
		depth.start();
	};
	
	void colorCB(std::function<void(cv::Mat3b)> cb) { colorListener.cb = cb; }

	void depthCB(std::function<void(cv::Mat1s)> cb) { depthListener.cb = cb; }

	operator bool() const { return depth.isValid() or color.isValid(); }
};

