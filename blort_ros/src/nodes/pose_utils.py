#!/usr/bin/env python

#
# Software License Agreement (Modified BSD License)
#
#  Copyright (c) 2012, PAL Robotics, S.L.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of PAL Robotics, S.L. nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

import rospy
import re
import subprocess
from multiprocessing import Process
from geometry_msgs.msg import Pose
import sys

def readPose(filename):
    f = open(filename, 'r')
    content = f.read()
    f.close()
    vects = re.findall(r'\[.*\]', content)
    pose = Pose()
    position = re.findall(r'[-0-9.]+', vects[0])
    orientation = re.findall(r'[-0-9.]+', vects[1])
    
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(orientation[0])
    pose.orientation.y = float(orientation[1])
    pose.orientation.z = float(orientation[2])
    pose.orientation.w = float(orientation[3])
    return pose

def readPoses(filename):
    poses = []
    f = open(filename, 'r')
    content = f.read()
    f.close()
    numbers = re.findall(r'[-0-9.]+', content)
    for i in range(0,len(numbers), 7):
        pose = Pose()
        pose.position.x = float(numbers[i])
        pose.position.y = float(numbers[i+1])
        pose.position.z = float(numbers[i+2])
        pose.orientation.x = float(numbers[i+3])
        pose.orientation.y = float(numbers[i+4])
        pose.orientation.z = float(numbers[i+5])
        pose.orientation.w = float(numbers[i+6])
        poses.append(pose)
    return poses

def poseDiff(pose1, pose2):
    result = Pose()
    result.position.x = abs(pose1.position.x - pose2.position.x)
    result.position.y = abs(pose1.position.y-pose2.position.y)
    result.position.z = abs(pose1.position.z-pose2.position.z)
    result.orientation.x = abs(pose1.orientation.x-pose2.orientation.x)
    result.orientation.y = abs(pose1.orientation.y-pose2.orientation.y)
    result.orientation.z = abs(pose1.orientation.z-pose2.orientation.z)
    result.orientation.w = abs(pose1.orientation.w-pose2.orientation.w)
    return result

def poseListMean(poseList):
    result = Pose()
    count = float(len(poseList))
    result.position.x = sum([act_pose.position.x for act_pose in poseList]) / count
    result.position.y = sum([act_pose.position.y for act_pose in poseList]) / count
    result.position.z = sum([act_pose.position.z for act_pose in poseList]) / count
    result.orientation.x = sum([act_pose.orientation.x for act_pose in poseList]) / count
    result.orientation.y = sum([act_pose.orientation.y for act_pose in poseList]) / count
    result.orientation.z = sum([act_pose.orientation.z for act_pose in poseList]) / count
    result.orientation.w = sum([act_pose.orientation.w for act_pose in poseList]) / count
    return result

def poseListAvgDiff(pose, poselist):
    diffs = [poseDiff(pose, act_pose) for act_pose in poselist]
    return poseListMean(diffs)

def poseListVariance(poseList):
    meanPose = poseListMean(poseList)
    return poseDiffList(meanPose, poseList)

def core():
    subprocess.call("roscore", executable="bash", shell=False)

def disparity():
    subprocess.call("roslaunch pal_disparity_segmentation disparity_check.launch", executable="bash", shell=False)

def blort():
    subprocess.call("roslaunch pal_blort blort_singleshot_disparity.launch", executable="bash", shell=False)

def disparity():
    subprocess.call("roslaunch pal_disparity_segmentation disparity_check.launch", executable="bash", shell=False)

def launch_procs():
    processes = [Process(target=core), Process(target=disparity), Process(target=blort)]
    for p in processes:
        p.start()
    return processes

def poseValidate(pose, pose_estimate, max_error):
    diff = poseDiff(pose, pose_estimate)
    if (diff.position.x - max_error.position.x < 0) and \
        (diff.position.y - max_error.position.y < 0) and \
        (diff.position.z - max_error.position.z < 0) and \
        (diff.orientation.x - max_error.orientation.x < 0) and \
        (diff.orientation.y - max_error.orientation.y < 0) and \
        (diff.orientation.z - max_error.orientation.z < 0) and \
        (diff.orientation.w - max_error.orientation.w < 0):
        return True
    else:
        return False

def validatePosesFromFile(pose, max_error, filename):
    poses = readPoses(filename)
    return [poseValidate(pose, act, max_error) for act in poses]

def geomQuatTo3x3Mat(quaternion):
    x2 = quaternion.x * quaternion.x;
    y2 = quaternion.y * quaternion.y;
    z2 = quaternion.z * quaternion.z;
    xy = quaternion.x * quaternion.y;
    xz = quaternion.x * quaternion.z;
    yz = quaternion.y * quaternion.z;
    wx = quaternion.w * quaternion.x;
    wy = quaternion.w * quaternion.y;
    wz = quaternion.w * quaternion.z;

    result =  [[1.0-(2.0*(y2 + z2)), 2.0*(xy - wz), 2.0*(xz + wy)], \
               [2.0*(xy + wz), 1.0-2.0*(x2 + z2), 2.0*(yz - wx)], \
               [2.0*(xz - wy), 2.0*(yz + wx), 1.0-(2.0*(x2 + y2))]]
    return result

