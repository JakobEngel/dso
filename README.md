# Stereo Direct Sparse Odometry


## Changes in this repository

* Created FullSystem/CoarseInitializerStereo for stereo initializationi - note that we stick with block matching since it is fast and the quality of the inverse depth will not affect dso horribly, since it solves for it
* Altered util/ImageAndExposure to carry two images and baseline
* Altered util/DatasetReader to read kiti stereo data

## Dependencies and how to Install Them (on debian-esque distros)

* Must have linux system (tested on ubuntu 16.04)	
* ROS 
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full	
```
* OpenCV

	Comes with ROS

* SuiteSparse, Eigen3, Boost
`$ sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev`

* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
```
$ git clone https://github.com/stevenlovegrove/Pangolin.git`
$ cd Pangolin
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release .. && make -j && sudo make install
```

## How to Compile

* Make a catkin workspace and clone this repo into it (or move the code into it from the Canvas submission)
```
$ mkdir catkin_ws && cd catkin_ws
$ mkdir src && cd src 
$ catkin_init_workspace
$ git clone https://github.com/rpng/dso.git # OR move the code here and unzip it
```
* Build the program using catkin and source the setup file
```
$ cd ..
$ catkin_make
$ source devel/setup.bash # or setup.zsh if you use zsh
```

## How to Run on KITTI Sequence 00 

* Download the dataset [here](https://drive.google.com/open?id=1u4aV863xfdwLZOhlHw1J0-hkjf5kUBMl) and unzip it
* Move the directory kitti_odom containing the directory called 00 into /path/to/dso 
* We provide a launch file for general use located at /path/to/dso/launch/kitti_general.launch
	* Note that if you did not move the dataset to /path/to/dso you can change the relative path in the launch file to run it
* Run dso!
```
$ cd /path/to/dso/launch
$ roslaunch kitti_generel.launch
```


## License
DSO was developed at the Technical University of Munich and Intel.
The open-source version is licensed under the GNU General Public License Version 3 (GPLv3).
For commercial purposes, we also offer a professional version, see
[http://vision.in.tum.de/dso](http://vision.in.tum.de/dso) for
details.

*All additional code added is released under the GNU General Public License Version 3 (GPLv3).*
