# DSO: Direct Sparse Odometry


## Changes in this repository

* Install with ROS catkin now
* Clone the repository into your ros workspace
* `sudo apt-get install libsuitesparse-dev libeigen3-dev libboost-all-dev`
* Ensure [Pangolin](https://github.com/stevenlovegrove/Pangolin) is globally installed
* Build the program using catkin
* Grab a dataset from the TUM website [link](https://vision.in.tum.de/data/datasets/mono-dataset)
* Extract the image.zip file if not building with ziplib
* Run the program using: `rosrun dso dso_dataset files=XXXX/images/ calib=XXXX/camera.txt gamma=XXXX/pcalib.txt vignette=XXXX/vignette.png`


## License
DSO was developed at the Technical University of Munich and Intel.
The open-source version is licensed under the GNU General Public License Version 3 (GPLv3).
For commercial purposes, we also offer a professional version, see
[http://vision.in.tum.de/dso](http://vision.in.tum.de/dso) for
details.

*All additional code added is released under the GNU General Public License Version 3 (GPLv3).*