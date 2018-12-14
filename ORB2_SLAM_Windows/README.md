# ORB_SLAM2_Windows
Modified version from https://github.com/Phylliida/orbslam-windows
- Incorporated OpenCV source code
- Slight modifications to scripts to try to simplify this process a little more
- Tested with Visual Studio 15 2017 Win64

Pre-required Installations:
 
1. Install Visual Studio 15 2017 Community from this site: https://visualstudio.microsoft.com/downloads/
2. Install CMake 3.12.4 this site: https://cmake.org/download/
3. Download opencv-3.4.3-vc14_vc15.exe from https://github.com/opencv/opencv/releases
- Run the executable and extract it to the "...orbslam-windows/Thirdparty/" folder 
- Add "...orbslam-windows\Thirdparty\opencv\build\x64\vc15\bin" to PATH via the OS Environment Variables

Building ORB Slam 2 on Windows

1. Build orbslam-windows/Thirdparty/DBoW2
- Run CMake GUI and set source code to orbslam-windows/Thirdparty/DBoW2 and where to build the binaries to orbslam-windows/Thirdparty/DBoW2/build
- Press Configure and choose Visual Studio 15 2017 Win64
- Press Generate
- Open the resulting project in the build directory in Visual Studio
- Change build type to Release (in white box up top, should initially say Debug)
- Right click on DBoW2 project -> Properties -> General: change Target Extension to .lib and Configuration Type to Static Library (.lib)
- Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)
- Build ALL_BUILD. You should get lots of warnings but no errors

2. Build orbslam-windows/Thirdparty/g2o
- Run CMake GUI and set source code to orbslam-windows/Thirdparty/g2o and where to build the binaries to orbslam-windows/Thirdparty/g2o/build
- Press Configure and choose Visual Studio 15 2017 Win64
- Press Generate
- Open the resulting project in the build directory in Visual Studio
- Change build type to Release (in white box up top, should initially say Debug)
- Right click on g2o project -> Properties -> General: change Target Extension to .lib and Configuration Type to Static Library (.lib)
- Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)
- Go to C/C++ -> Preprocessor and press the dropdown arrow in the Preprocessor Definitions, then add a new line with WINDOWS on it (no underscore), then press OK, then Apply
- Build ALL_BUILD.

3. Build orbslam-windows/Thirdparty/Pangolin
- Run CMake GUI and set source code to orbslam-windows/Thirdparty/Pangolin and where to build the binaries to orbslam-windows/Thirdparty/Pangolin/build
- Press Configure and choose Visual Studio 15 2017 Win64
- Press Generate
- Open the resulting project in the build directory in Visual Studio
- Change build type to Release (in white box up top, should initially say Debug)
- Build ALL_BUILD. You'll have an error by project testlog that says "cannot open input file 'pthread.lib'" but that doesn't matter cause we don't use testlog. Everything else should build fine, i.e., you should have
========== Build: 18 succeeded, 1 failed, 0 up-to-date, 0 skipped ==========

4. Build orbslam-windows
- Run CMake GUI and set source code to orbslam-windows and where to build the binaries to orbslam-windows/build
- Press Configure and choose Visual Studio 15 2017 Win64
- Press Generate
- Open the resulting project in the build directory in Visual Studio
- Change build type to Release (in white box up top, should initially say Debug)
- Right click on ORB_SLAM2 project -> Properties -> General: change Target Extension to .lib and Configuration Type to Static Library (.lib)
- Go to C/C++ Tab -> Code Generation and change Runtime Library to Multi-threaded (/MT)

- Right click on the ORB_SLAM2 project (NOT ALL_BUILD) and click Build
- If you're lucky, that will take few minutes then successfully build!

If you want to build any of the examples (such as mono_euroc), do the following:

- Right click on that project and go to Properties -> C/C++ -> Code Generation, and change Runtime Library to Multi-threaded (/MT). Then press apply
- Right click on it and press build

Then you will find them, say if you do mono_, in (orbslam-windows\Examples\Monocular\Release)
