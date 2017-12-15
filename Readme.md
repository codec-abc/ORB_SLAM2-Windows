# ORB SLAM 2 Windows

## Building

First build OpenCV (version 3.3.1) from source with the following options:

* Disable static runtime
* Disable OpenMP
* Disable Intel IPP
* Generate OpenCV World
* Disable tests and documentation generation (not mandatory only save times)

Once generated a Visual Studio solution using CMake. Set the ```OpenCVIncludeDir``` variable to the path where the includes folder is.