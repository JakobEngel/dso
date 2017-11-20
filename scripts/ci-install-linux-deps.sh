#!/usr/bin/env bash

# Script to install dependencies, intended for CI builds, in particular for Travis

set -x # echo on
set -e # exit on error

ROOT_DIR="$PWD"

sudo apt-get update -qq

# basic dependencies (needed in minimalistic docker containers, but not travis)
#sudo apt-get install -qq build-essential git cmake wget pkg-config

# get more recent cmake for e.g. CMAKE_CXX_COMPILER_LAUNCHER
cd "$ROOT_DIR"
wget https://cmake.org/files/v3.9/cmake-3.9.0-Linux-x86_64.sh
chmod +x cmake-3.9.0-Linux-x86_64.sh
set +x # echo off
sudo ./cmake-3.9.0-Linux-x86_64.sh  --skip-license --prefix=/usr/local
set -x # echo on
sudo update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 1 --force
cmake --version

# get recent libeigen, since the one on 14.04 gives lots of warnings
cd "$ROOT_DIR"
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz
tar -xf 3.3.4.tar.gz
cd eigen-eigen-*
mkdir build
cd build/
cmake ..
sudo make install -j1

# install pangolin from source
cd "$ROOT_DIR"
# skip libeigen3-dev, since we installed manually above
sudo apt-get install -qq libglew-dev libc++-dev
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir Pangolin/build
cd Pangolin/build
cmake -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_BUILD_TYPE=$BUILD_TYPE ..
make

# deps for DSO
# skip libeigen3-dev, since we installed manually above
sudo apt-get install -qq libboost-all-dev libopencv-dev

# install libzip (for Ubuntu 16.04 and up, libzip-dev should be ok)
cd "$ROOT_DIR"
sudo apt-get install -qq zlib1g-dev
cd thirdparty
tar -zxvf libzip-1.1.1.tar.gz
cd libzip-1.1.1/
./configure
make
sudo make install
sudo cp lib/zipconf.h /usr/local/include/zipconf.h   # (no idea why that is needed).
