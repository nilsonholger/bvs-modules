BVS (BASE) MODULES
==================

* **AndroidCV:** Display module for the android client.
* **CalibrationCV:** Calibrate cameras (nodes), currently only extrinsic and intrinsic calibration for a stereo camera setup is supported.
* **CaptureCV:** Common capture/conversion module, provides access to streams from attached cameras as well as image/video conversions.
* **ExampleCV:** Example of how to use the framework.
* **KinectXLite:** Manuel Martinez's header only Kinect driver.
* **OpenNIXLite:** Manuel Martinez's header only OpenNI wrapper [requires OpenNI2].
* **StereoCVCUDA:** Wrapper for OpenCV's CUDA stereo capabilities.
* **StereoELAS:** Wrapper for Andreas Geiger's excellent libELAS stereo library.
* **WebStreamer:** Small MJPEG web(socket) stream to view on a browser, built upon Manuel Martinez's awesome uSnippets.

**NOTE:**

Modules ending in or containing `CV` use OpenCV for their core functionality.
Most, if not all, modules use OpenCV's `cv::Mat` for passing data (images) around.

**NOTE:**

StereoELAS uses Andreas Geiger's libELAS (www.cvlibs.net). It is provided under its own licence, which can be found in `StereoELAS/elas/LICENSE.TXT`.
KinectXLite and OpenNIXLite are licensed under LGPLv3 (https://github.com/uSnippets/uSnippets).
