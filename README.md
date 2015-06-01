This is the software about indoor scene analysis and reconstruction based on "InfiniTAM".

InfiniTAM: http://www.robots.ox.ac.uk/~victor/infinitam/


1. Requirements

Several 3rd party libraries are needed for compiling this code. The given
version numbers are checked and working, but different versions might be
fine as well. Some of the libraries are optional, and skipping them will
reduce functionality.

  - cmake (e.g. version 2.8.10.2)
    REQUIRED for Linux, unless you write your own build system
    OPTIONAL for MS Windows, if you use MSVC instead
    available at http://www.cmake.org/

  - OpenGL / GLUT (e.g. freeglut 2.8.0)
    REQUIRED for the visualisation
    the library should run without
    available at http://freeglut.sourceforge.net/

  - CUDA (e.g. version 6.0)
    OPTIONAL but REQUIRED for all GPU accelerated code
    at least with cmake it is still possible to compile the CPU part without
    available at https://developer.nvidia.com/cuda-downloads

  - OpenNI (e.g. version 2.2.0.33)
    OPTIONAL but REQUIRED to get live images from the suitable hardware
    also make sure you have freenect/OpenNI2-FreenectDriver if you need it
    available at http://structure.io/openni

  - doxygen (e.g. version 1.8.2)
    OPTIONAL, builds a nice reference manual
    available at http://www.doxygen.org/
	
  - Boost
  
  - Eigen
  
  - flann
  
  - freeglut
  
  - GeometricTool
  
  - opencv
  
  - PCL
  
  - qhull
  
  - trimesh
  
  - vcglib
  
  - VTK
  
  

