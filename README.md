This is based on a fork of the original tum-vision/lsd_slam.

Removed the ROS dependency, and I'm pushing towards a more batch oriented processing. To serve my evil purposes, of course.



Dependencies:
- g2o 41b0be7 from https://github.com/RainerKuemmerle/g2o
- Eigen3.3-beta1, which is ce5a455b34c0
- boost 1.59.0
- glog 0.3.4
- gflags 2.1.2
- opencv 3.1.0
- suit-parse 4.4.4
- hdf5 1.8.16 installed with -c++11 options from homebrew/science/hdf5

Other versions might work too.