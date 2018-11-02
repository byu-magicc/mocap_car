# kb_red

To build you'll need to either install the librealsense drivers (not necessary) or build with `catkin_make -DCATKIN_BLACKLIST_PACKAGES="realsense2_camera"` (recommended)

When cloning anew, don't forget to clone the submodules as well:
`git submodule update --init --recursive`

# Dependencies

`ros-kinetic-camera-calibration-parsers`

`ros-kinetic-libg2o`

`libsuitesparse-dev`
