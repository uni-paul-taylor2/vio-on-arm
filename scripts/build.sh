# this is intended for kimera at first then would be edited for actual use in this repository
sudo apt install -y cmake libboost-all-dev build-essential libtbb-dev libgflags-dev libgoogle-glog-dev
sudo apt install -y unzip pkg-config libjpeg-dev libpng-dev libtiff-dev libvtk7-dev libgtk-3-dev libpartemis-dev libatlas-base-dev gfortran

cd $HOME
mkdir cmake_repositories
cd cmake_repositories

# install gtsam
if [ ! -d gtsam ]; then
  git clone git@github.com:borglab/gtsam.git
  cd gtsam
  git checkout 4.2
  mkdir build
  cd build
  cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF ..
  sudo make -j $(nproc) install
  cd ..
fi

# install opencv
if [ ! -d opencv ]; then
  git clone https://github.com/opencv/opencv.git
  cd opencv
  git checkout tags/4.2
  #there is a way later version, 4.11 at the time of this commit though
  mkdir build
  cd build
  cmake -DWITH_VTK=On -DWITH_TBB=On ..
  sudo make -j $(nproc) install
  cd ..
fi

# install opengv
if [ ! -d opengv ]; then
  git clone https://github.com/laurentkneip/opengv.git
  cd opengv
  mkdir build
  cd build
  cmake .. -DEIGEN_INCLUDE_DIR=/usr/local/include/gtsam/gtsam/3rdparty/Eigen -DEIGEN_INCLUDE_DIRS=/usr/local/include/gtsam/gtsam/3rdparty/Eigen
  sudo make -j $(nproc) install
  cd ..
fi

# install kimera-rpgo
if [ ! -d Kimera-RPGO ]; then
  git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
  cd Kimera-RPGO
  mkdir build
  cd build
  cmake ..
  sudo make -j $(nproc)
  cd ..
fi

# install kimera-vio
if [ ! -d Kimera-VIO ]; then
  git clone git@github.com:MIT-SPARK/Kimera-VIO.git Kimera-VIO
  cd Kimera-VIO
  mkdir build
  cmake ..
  sudo make -j $(nproc)
  cd ..
fi
