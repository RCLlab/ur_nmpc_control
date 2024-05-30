#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
from std_msgs.msg import Float64MultiArray

x_temp = 0.0
y_temp = 0.7
z_temp = 0.0
point_array_temp = [0.478848684775825+x_temp,    0.229499373901787+y_temp,	1.68485092992956+z_temp,
                    0.427716972797897+x_temp,	 0.156512773607464+y_temp,	1.34040111476562+z_temp,
                    0.369315034866102+x_temp,	 0.309845500752365+y_temp,	1.49875708954259+z_temp,
                    0.557165971235294+x_temp,	 0.040544077757588+y_temp,	1.48606823051146+z_temp,
                    0.419712934985229+x_temp,	 0.399070259311829+y_temp,	1.61120853354152+z_temp,
                    0.681109554751682+x_temp,	 0.023558220982218+y_temp,	1.57264652980510+z_temp,
                    0.470110835104355+x_temp,	 0.488295017871293+y_temp,	1.72365997754045+z_temp,
                    0.805053138268070+x_temp,	 0.006572364206847+y_temp,	1.65922482909874+z_temp,
                    0.592559519594503+x_temp,	 0.467311156719235+y_temp,	1.83400793485003+z_temp,
                    0.844196693313332+x_temp,	 0.107177374615454+y_temp,	1.78554619872287+z_temp,
                    0.730652473227315+x_temp,	 0.450733840669821+y_temp,	1.94163505027114+z_temp,
                    0.889527627363086+x_temp,	 0.210151806343637+y_temp,	1.92122778328762+z_temp,
                    0.457502242864183+x_temp,	 0.169519703432083+y_temp,	1.53470799130878+z_temp,
                    0.407639420232935+x_temp,	 0.150118041942872+y_temp,	1.1672089686425+z_temp]

class ENV:
    def __init__(self):
        self.pub = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)
        self.iter = 0

    def step(self):
        point_array = [0]*42
        for a in range(14):
            point_array[3*a] = point_array_temp[3*a]
            point_array[3*a+1] = point_array_temp[3*a+1]
            point_array[3*a+2] = point_array_temp[3*a+2]
        obstacle_data = Float64MultiArray()
        obstacle_data.data = point_array
        self.pub.publish(obstacle_data)

if __name__ == '__main__':
    rospy.init_node("human_poses_provider", anonymous=True)
    env = ENV()
    cond_temp=0
    rate = rospy.Rate(125) #hz
    while not rospy.is_shutdown():
        env.step()
        rate.sleep()
