START_DIR=$(pwd)
sudo apt-get update
sudo apt upgrade
sudo apt install -y libboost-all-dev build-essential libtbb-dev libgflags-dev libgoogle-glog-dev libavcodec-dev libavformat-dev libswscale-dev \
  unzip pkg-config libjpeg-dev libpng-dev libtiff-dev libvtk7-dev libgtk-3-dev libparmetis-dev libatlas-base-dev gfortran ffmpeg \
  python3-dev python3-numpy libeigen3-dev libopencv-dev libopencv-contrib-dev libmetis-dev xvfb python3 python3-pip python3-tk

cd $HOME
mkdir cmake_repositories
cd cmake_repositories

# install gtsam
if [ ! -d gtsam ]; then
  #git clone git@github.com:borglab/gtsam.git
  #above notation was causing some concerning looking prompts so back to the default method
  git clone https://github.com/borglab/gtsam.git
  cd gtsam
  git checkout tags/4.2
  mkdir build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_USE_SYSTEM_METIS=ON
  sudo make -j $(nproc) install
  cd ../..
fi

cd $START_DIR
mkdir -p build
cd build
cmake ..
sudo make -j $(nproc)
cd ..
#now to run ./build/exported_lib.so or what not (TODO)
