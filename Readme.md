BVS (BASE) MODULES
==================

* **AndroidCV**: Display module for the android client.
* **CalibrationCV**: Used to calibrate cameras (nodes). Currently, only extrinsic and intrinsic calibration for stereo cameras is supported.
* **CaptureCV**: General capture module. Can access streams from attached camaras, also read and write from/to images and videos.
* **ExampleCV**: Example module providing an example of how to use the framework.
* **StereoCVCUDA**: Wrapper for OpenCV's stereo capabilities.
* **StereoELAS**: Wrapper for Andreas Geiger's excellent libELAS stereo library.

**NOTE:**

Modules ending/containing `CV` **require** OpenCV.

**NOTE:**

StereoELAS uses Andreas Geiger's libELAS (www.cvlibs.net). It is provided under its own licence, which can be found in `StereoELAS/elas/LICENSE.TXT`.
