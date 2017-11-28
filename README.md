# DSO: Direct Sparse Odometry


## Changes in this repository

* Install with ROS catkin now
* Clone the repository into your ros workspace
* `sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev`
* Ensure [Pangolin](https://github.com/stevenlovegrove/Pangolin) is globally installed
* Build the program using catkin
* Grab a dataset from the KITTI website [link](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
    * Will want grayscale images
    * If you want to create your own config, download the config set
    * Config file has an extra line of the external transform between stereo cameras
* Please take a look at the provided launch files
* Run the program using: `rosrun dso dso_dataset filesL=imagesLEFT/ filesR=imagesRIGHT/ calib=config/kitti_odom.txt mode=2 prefetch=0`


## License
DSO was developed at the Technical University of Munich and Intel.
The open-source version is licensed under the GNU General Public License Version 3 (GPLv3).
For commercial purposes, we also offer a professional version, see
[http://vision.in.tum.de/dso](http://vision.in.tum.de/dso) for
details.

*All additional code added is released under the GNU General Public License Version 3 (GPLv3).*