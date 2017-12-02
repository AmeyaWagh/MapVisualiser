#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import Range
import matplotlib.pyplot as plt
import datetime

data_len=50
refreshRate = 5
global starttime
starttime = datetime.datetime.now()

global sensor1_arr
sensor1_arr = [0]

global sensor2_arr
sensor2_arr = [0]

global sensor3_arr
sensor3_arr = [0]

global sensor4_arr
sensor4_arr = [0]

global sensor5_arr
sensor5_arr = [0]


def callback1(data):
    rospy.loginfo(rospy.get_caller_id() + 'sensor 1 {}'.format(data.range))
    global sensor1_arr
    sensor1_arr.append(data.range)
    sensor1_arr = buffResize(sensor1_arr)
    plotter()


def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + 'sensor 2 {}'.format(data.range))
    global sensor2_arr
    sensor2_arr.append(data.range)
    sensor2_arr = buffResize(sensor2_arr)
    plotter()

def callback3(data):
    rospy.loginfo(rospy.get_caller_id() + 'sensor 3 {}'.format(data.range))
    global sensor3_arr
    sensor3_arr.append(data.range)
    sensor3_arr = buffResize(sensor3_arr)
    plotter()

def callback4(data):
    rospy.loginfo(rospy.get_caller_id() + 'sensor 4 {}'.format(data.range))
    global sensor4_arr
    sensor4_arr.append(data.range)
    sensor4_arr = buffResize(sensor4_arr)
    plotter()

def callback5(data):
    rospy.loginfo(rospy.get_caller_id() + 'sensor 5 {}'.format(data.range))
    global sensor5_arr
    sensor5_arr.append(data.range)
    sensor5_arr = buffResize(sensor5_arr)
    plotter()



def buffResize(sensor_arr):
    while len(sensor_arr)>data_len:
        sensor_arr.pop(0)
    return sensor_arr    


def plotter():
    pass
    # global starttime
    # if (datetime.datetime.now()-starttime).seconds > refreshRate:
    #     plt.close()
    #     plt.plot(sensor1_arr)
    #     plt.plot(sensor2_arr)
    #     plt.plot(sensor3_arr)
    #     plt.plot(sensor4_arr)
    #     plt.plot(sensor5_arr)
            
    #     plt.show()
    #     starttime = datetime.datetime.now()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('sensor_range1', Range, callback1)
    rospy.Subscriber('sensor_range2', Range, callback2)
    rospy.Subscriber('sensor_range3', Range, callback3)
    rospy.Subscriber('sensor_range4', Range, callback4)
    rospy.Subscriber('sensor_range5', Range, callback5)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    listener()
