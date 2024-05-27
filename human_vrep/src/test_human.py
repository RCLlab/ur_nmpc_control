#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
from std_msgs.msg import Float64MultiArray
pos = []

# Change the directory to the location of human dataset file
pos_183 = 'Participant_8410_Setup_A_Seq_1_Trial_3.xsens.bvh.csv'
pos_183 = pd.read_csv(pos_183, quoting=csv.QUOTE_NONNUMERIC)
pos_183 = pos_183.to_numpy()

class ENV:
    def __init__(self):
        """
        Initialize the class by setting up a ROS subscriber & publisher
        """

        rospy.Subscriber('/flag', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)
        self.iter = 0
        self.condition_h = [0]*2

    def callback(self, data):
        """
        Record the data received via ROS subscriber
        """

        self.condition_h = data.data[0:2]

    def check_condition(self):
        return [int(self.condition_h[0]),self.condition_h[1]]

    def step(self,i):
        """
        Process and send human data to the controller for a specific step.
        """
        point_array = [0]*45
        for a in range(14):
            point_array[3*a] = (pos_183[i][3*a])
            point_array[3*a+1] = (pos_183[i][3*a+1])+0.7
            point_array[3*a+2] = (pos_183[i][3*a+2])
        point_array[43] = 1
        point_array[44] = 1
        obstacle_data = Float64MultiArray()
        obstacle_data.data = point_array
        self.pub.publish(obstacle_data)

if __name__ == '__main__':
    rospy.init_node("human_poses_provider", anonymous=True)
    env = ENV()
    i = 0
    cond_temp=0
    rate = rospy.Rate(125) #hz
    msg = rospy.wait_for_message("/flag", Float64MultiArray)
    if(msg):
        while not rospy.is_shutdown():
            condition_h = env.check_condition()
            if condition_h[1]==1:
                time.sleep(0.02)
                i=12500
            else:
                env.step(i)
                print(condition_h,i)
                i+=1
            rate.sleep()
