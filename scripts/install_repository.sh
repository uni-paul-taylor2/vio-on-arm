#you can open a vm, reach here, copy this and paste it straight in the terminal :D
#startup stuff AND installing cmake

sudo apt upgrade

sudo apt install gcc

sudo apt install g++

sudo apt install build-essential

sudo apt install libssl-dev

sudo apt-get install libboost-all-dev

sudo snap install cmake --classic

#if this specific line above doesn't work, open linux "App Center" and search for and install cmake there



#installing gtsam stuff

sudo apt install libopencv-dev

sudo add-apt-repository ppa:borglab/gtsam-develop

sudo apt update

sudo apt install libgtsam-dev libgtsam-unstable-dev

sudo apt install libeigen3-dev



#installing git now

sudo add-apt-repository ppa:git-core/ppa

sudo apt install git

git clone --single-branch --branch dev https://github.com/Virtana/vio-on-arm.git

cd vio-on-arm



#now for building

mkdir build

cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
