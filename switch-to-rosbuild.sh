#!/bin/sh

oldir=`pwd`
perception_blort_dir=`dirname $0`
cd $perception_blort_dir/siftgpu
ln -sfv CMakeLists.rosbuild.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort_ros_msgs
ln -sfv CMakeLists.rosbuild.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort
ln -sfv CMakeLists.rosbuild.txt CMakeLists.txt
cd $oldir
cd $perception_blort_dir/blort_ros
ln -sfv CMakeLists.rosbuild.txt CMakeLists.txt
cd cfg
ln -sfv Detector.cfg.rosbuild Detector.cfg
ln -sfv Tracker.cfg.rosbuild Tracker.cfg
cd $oldir
cd $perception_blort_dir
ln -sfv CMakeLists.rosbuild.txt CMakeLists.txt
cd $oldir
