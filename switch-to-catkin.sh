#!/bin/sh

oldir=`pwd`
perception_blort_dir=`dirname $0`
cd $perception_blort_dir/siftgpu
ln -sfv CMakeLists.catkin.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort_ros_msgs
ln -sfv CMakeLists.catkin.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort
ln -sfv CMakeLists.catkin.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort_ros
ln -sfv CMakeLists.catkin.txt CMakeLists.txt
cd cfg
ln -sfv Detector.cfg.catkin Detector.cfg
ln -sfv Tracker.cfg.catkin Tracker.cfg
cd $oldir
cd $perception_blort_dir
rm -f CMakeLists.txt
cd $oldir
