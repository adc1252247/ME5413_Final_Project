I had some issues - I did not use full installation, I suspect this is the reason for the issue

```txt
-- Could NOT find rviz (missing: rviz_DIR)
-- Could not find the required component 'rviz'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "rviz" with any of
  the following names:

    rvizConfig.cmake
    rviz-config.cmake

  Add the installation prefix of "rviz" to CMAKE_PREFIX_PATH or set
  "rviz_DIR" to a directory containing one of the above files.  If "rviz"
  provides a separate development package or SDK, be sure it has been
  installed.
Call Stack (most recent call first):
  interactive_tools/CMakeLists.txt:12 (find_package)


-- Configuring incomplete, errors occurred!

```

```sh
sudo apt-get install -y ros-noetic-rviz
```

```sh
sudo apt-get install -y ros-noetic-gazebo-ros
sudo apt-get install -y ros-noetic-gazebo-plugins
sudo apt-get install -y ros-noetic-gazebo-msgs
```

```sh
sudo apt-get install -y \
  ros-noetic-sick-tim \
  ros-noetic-lms1xx \
  ros-noetic-velodyne-description \
  ros-noetic-pointgrey-camera-description \
  ros-noetic-jackal-control \
  ros-noetic-flir-camera-description
```

```sh
# Install teleop_twist_keyboard
sudo apt-get install -y ros-noetic-teleop-twist-keyboard

# Install jsk_rviz_plugins (includes IMU plugin)
sudo apt-get install -y ros-noetic-jsk-rviz-plugins

# Also install related tools
sudo apt-get install -y ros-noetic-rqt-common-plugins
```

```sh
sudo apt install -y ros-noetic-gazebo-ros-control

```

```sh
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-plugins

```

```sh
sudo apt-get install ros-noetic-jackal-navigation

```

```sh
sudo apt-get install ros-noetic-rviz-imu-plugin

```