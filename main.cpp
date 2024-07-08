#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "markerDetection.cpp"

int main()
{
    std::cout << "Hello, World!" << std::endl;
    std::cout << CV_VERSION;
    detectMarkers();
    return 0;
}

/*
git clone --filter=blob:none https://github.com/opencv/opencv.git

cd opencv && git checkout tags/4.6.0

git clone --filter=blob:none  https://github.com/opencv/opencv_contrib.git

cd opencv_contrib/ &&  git checkout tags/4.6.0

cd opencv/ && mkdir build && cd build && \

cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/home/zakareeyah/opencv \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D ENABLE_FAST_MATH=ON \
-D BUILD_opencv_java=OFF \
-D BUILD_ZLIB=ON \
-D BUILD_TIFF=ON \
-D WITH_GTK=ON \
-D WITH_FFMPEG=ON \
-D WITH_1394=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D OPENCV_PC_FILE_NAME=opencv4.pc \
-D OPENCV_ENABLE_NONFREE=ON \
-D WITH_GSTREAMER=ON \
-D WITH_V4L=ON \
-D WITH_QT=ON \
-D WITH_OPENGL=ON \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules/ \
-D BUILD_EXAMPLES=ON ..

make -j4
*/