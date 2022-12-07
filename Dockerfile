FROM ubuntu:20.04

WORKDIR /app
ENV preset=2
ENV mode=1
ENV nolog=0
ENV reverse=0
ENV nomt=1
ENV speed=0
ENV nogui=1
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update  && \
    apt install -y git
RUN git clone https://github.com/IldarGreat/dso.git
RUN apt-get install -y build-essential \
    libsuitesparse-dev libeigen3-dev libboost-all-dev \
    libopencv-dev 

WORKDIR /app
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.6
WORKDIR /app/Pangolin
RUN apt-get install -y cmake && \
    apt-get clean && \
    apt install -y libglew-dev && \
    apt-get install -y libegl1-mesa-dev

RUN cmake -B build && \
    cmake --build build

RUN apt-get install -y zlib1g-dev
WORKDIR /app/dso/thirdparty/
RUN tar -zxvf libzip-1.1.1.tar.gz
WORKDIR /app/dso/thirdparty/libzip-1.1.1/
RUN ./configure && \
    make && \
    cp lib/zipconf.h /usr/local/include/zipconf.h

RUN git submodule update --init

WORKDIR /app/dso/build
RUN cmake .. && \
    make

ENTRYPOINT ./bin/dso_dataset files=set/sequence calib=set/camera.txt mode=${mode} preset=${preset} nolog=${nolog} reverse=${reverse} nomt=${nomt} speed=${speed} nogui=${nogui}
