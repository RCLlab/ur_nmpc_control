#!/usr/bin/env python
import rospy
import time
import pandas as pd
import csv
from std_msgs.msg import Float64MultiArray

human_pos = 'moving_human_data.csv'
human_pos = pd.read_csv(human_pos, quoting=csv.QUOTE_NONNUMERIC)
human_pos = human_pos.to_numpy()

class ENV:
    def __init__(self):
        rospy.Subscriber('/flag', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/Obstacle/human_spheres', Float64MultiArray, queue_size=1)
        self.iter = 0
        self.condition_h = [0]*2

    def callback(self, data):
        self.condition_h = data.data[0:2]

    def check_condition(self):
        return [int(self.condition_h[0]),self.condition_h[1]]

    def step(self,i):
        point_array = [0]*42
        for a in range(14):
            point_array[3*a] = (human_pos[i][3*a])-0.5
            point_array[3*a+1] = (human_pos[i][3*a+1]) + 1.0
            point_array[3*a+2] = (human_pos[i][3*a+2]) - 1.2
        obstacle_data = Float64MultiArray()
        obstacle_data.data = point_array
        self.pub.publish(obstacle_data)

if __name__ == '__main__':
    rospy.init_node("human_poses_provider", anonymous=True)
    env = ENV()
    i = 1
    cond_temp=0
    rate = rospy.Rate(125) #hz
    msg = rospy.wait_for_message("/flag", Float64MultiArray)
    if(msg):
        while not rospy.is_shutdown():
            condition_h = env.check_condition()
            if condition_h[1]==1:
                time.sleep(0.02)
                i=1
            else:
                env.step(i)
                print(condition_h,i)
                i+=1
            rate.sleep()
