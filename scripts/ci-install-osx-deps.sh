#!/usr/bin/env bash

# Script to install dependencies, intended for CI builds, in particular for Travis

set -x # echo on
set -e # exit on error

# Travis suggests to not call brew update
brew install boost eigen glew opencv libzip cmake pkgconfig

brew install ccache
export PATH="/usr/local/opt/ccache/libexec:$PATH"
whereis ccache

git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir Pangolin/build
cd Pangolin/build/
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_COMPILER_LAUNCHER=ccache ..
make
make install -j1

