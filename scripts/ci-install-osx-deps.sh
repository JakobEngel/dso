#!/usr/bin/env bash

# Script to install dependencies, intended for CI builds, in particular for Travis

set -x # echo on
set -e # exit on error

brew update

# The following are already present in the travis image.
# We can probably live with slightly outdated formulas, so skip the upgrade
# which takes very long (in particular boost).
#brew upgrade cmake pkgconfig boost

# remove numpy, which is present in the travis image.
# It will be installed from brew as opencv dependency, which fails if already present.
PIP=`which pip` || PIP=pip2    # newer osx travis images don't have pip executable
/usr/bin/yes | $PIP uninstall numpy > /dev/null  # 2>&1

brew install eigen glew opencv libzip

brew install ccache
export PATH="/usr/local/opt/ccache/libexec:$PATH"
whereis ccache

git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir Pangolin/build
cd Pangolin/build/
cmake -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DCMAKE_CXX_COMPILER_LAUNCHER=ccache ..
make
make install -j1

