#!/usr/bin/env python
import os
import stream_tee as stream_tee
# import __main__ as main
import rospy
from std_msgs.msg import Float64MultiArray, String
import time
import numpy as np
from stream_tee import write_mat


class ENV:
    def __init__(self,run_name,init_poses):
        rospy.Subscriber('/HighController/mpc_high_positions', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/poses', Float64MultiArray, queue_size=1)
        self.run_name = run_name
        self.i = 0
        self.init_poses = init_poses
        self.observation = [0]*7
    def callback(self, data):
        self.observation = data.data[0:7]
        self.i+=1

    def step(self,iteration):
        pub_data = Float64MultiArray()
        pub_data.data = self.init_poses[iteration]
        self.pub.publish(pub_data)
        self.joint_poses = self.observation[0:7]
        
    def save_log(self):
        rec_dir = '/home/robot/workspaces/Big_Data/'
        os.chdir(rec_dir)
        write_mat('Tests/set_feasibility/10'+self.run_name,
                        {'joint_positions': self.joint_poses},
                        str(save_iter))    


if __name__ == '__main__':
    rospy.init_node("mpc_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    t = time.time()
    i = 0
    save_iter = 0
    rate = rospy.Rate(10) 
    goal = [0.0, -2.0, -1.22, -1.518, -1.588, 0.5]
    theta_1 = np.linspace(goal[0] - 3.14, 3.14 + goal[0], 50)
    theta_2 = np.linspace(goal[1] - 3.14, 3.14 + goal[1], 50)

    init_poses=np.array([goal]*2500)
    for i in range(len(theta_1)-1):
        for j in range(len(theta_2)-1):
            itte = i*len(theta_2)+j
            # print(i,j,itte)
            init_poses[itte][0] = theta_1[i]
            init_poses[itte][1] = theta_2[j]
    env = ENV(run_name,init_poses)
    
    while save_iter<len(init_poses):
        env.step(save_iter)
        save_iter+=1
        print(save_iter,init_poses[save_iter])
        if save_iter>len(init_poses)-2:
            env.save_log()
        rate.sleep()
    env.save_log()
    
        