#!/bin/bash

catkin_workspace=$(cd "$(dirname $0)"; pwd)
ros_install_prefix="/opt/ros"

if test ! -e ${ros_install_prefix}; then
  echo "[error] no ros distribution installed"
  exit 1
fi

if test -e ${ros_install_prefix}/indigo; then
  ros_version="indigo"
  echo "[debug] ros version: ${ros_version}"
fi

if test -e ${ros_install_prefix}/kinetic; then
  ros_version="kinetic"
  echo "[debug] ros version: ${ros_version}"
fi

if test ! -e ${catkin_workspace}/src/CMakeLists.txt; then
  cd ${catkin_workspace}/src && command catkin_init_workspace
fi

cd ${catkin_workspace} && command catkin_make && source ${catkin_workspace}/devel/setup.bash && rospack profile

