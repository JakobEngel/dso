sudo apt-get install -y libsuitesparse-dev libeigen3-dev libboost-all-dev libopencv-dev libglew-dev libegl1-mesa-dev cmake zlib1g-dev curl lsb-release
user=ildar
cd /home/$user
mkdir app
cd app
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.6
cd Pangolin
mkdir build
cd build
cmake ..
make

cd /home/$user/app
git clone https://github.com/IldarGreat/dso.git
cd dso
git submodule update --init
mkdir build
cd build
cmake ..
make 
