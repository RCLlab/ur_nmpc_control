#!/usr/bin/env python
import os
import stream_tee as stream_tee
import rospy
from std_msgs.msg import Float64MultiArray, String
import time
from stream_tee import write_mat
from initialization import set_init_pose
class ENV:
    def __init__(self,run_name,term_mode):
        rospy.Subscriber('/info', Float64MultiArray, self.callback)
        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self.flag_pub = rospy.Publisher('/flag', Float64MultiArray, queue_size=1)
        self.run_name = run_name
        self.init_poses = [2.619, -0.958, -1.22, -1.518, -1.588, 0.5] #static human
        # self.init_poses = [2.9, -1.2, -1.22, -1.518, -1.588, 0.5] #moving human
        self.term_mode = term_mode
        self.i = 0
        self.forcestop = 0
        self.first = 0
        self.init_log_variables()
        self.total = 0
        self.hello_str=[-1,0]
        self.max_diff = 10
        self.t_total = time.time()
        
    def callback(self, data):
        self.observation = data.data[0:116]

    def done(self):
        arrive = False
        if self.max_diff<0.02 or self.forcestop>400:
            print("-----Arrived------", self.max_diff)
            arrive = True

        return arrive

    def step(self):
        self.forcestop+=1

        vel = [self.observation[48],self.observation[49],self.observation[50],self.observation[51],self.observation[52],self.observation[53]]
        
        c_vel = [vel[0],vel[1],vel[2],vel[3],vel[4],vel[5]]
        hello_str = "speedj(["+str(c_vel[0])+","+str(c_vel[1])+","+str(c_vel[2])+","+str(c_vel[3])+","+str(c_vel[4])+","+str(c_vel[5])+"],"+"5.0"+",0.1)" 

        self.pub.publish(hello_str)

        self.joint_poses.append(self.observation[0:6]) 
        self.human_poses.append(self.observation[6:48])
        self.mpc_sol.append(self.observation[48:64])
        self.low_goal.append(self.observation[64:70])
        self.from_high_controller.append(self.observation[70:102])
        self.max_diff = self.observation[101]
        self.lin_vel_limit.append(self.observation[102:109])
        self.ctv_linear.append(self.observation[109:116])
    
    def reset(self): 
        self.forcestop = 0
        print("reset")
        self.hello_str[1] = 1
        pub_data = Float64MultiArray()
        pub_data.data = self.hello_str
        self.flag_pub.publish(pub_data)
        time.sleep(1)
        self.save_log(self.i)
        self.i+=1
        print(self.init_poses)
        set_init_pose(self.init_poses[0:6], 10)
        time.sleep(0.5)
        self.init_log_variables()
        self.hello_str[0]+= 1
        self.hello_str[1] = 0
        pub_data.data = self.hello_str
        self.flag_pub.publish(pub_data)
        time.sleep(1)
        self.step()
    
    def init_log_variables(self):
        self.observation = [1]*116
        self.joint_poses = []
        self.human_poses = []
        self.mpc_sol = []
        self.low_goal = []
        self.from_high_controller = []
        self.lin_vel_limit = []
        self.ctv_linear = []
        self.diff = 10

    def save_log(self,save_iter):
        rec_dir = '/home/robot/workspaces/Big_Data/'
        os.chdir(rec_dir)
        write_mat('Tests/'+self.term_mode + '/'+self.run_name,
                        {'joint_positions': self.joint_poses,
                        'human_poses':self.human_poses,
                        'mpc_sol':  self.mpc_sol,
                        'low_goal': self.low_goal,
                        'from_high_controller':self.from_high_controller,
                        'lin_vel_limit': self.lin_vel_limit,
                        'ctv_linear': self.ctv_linear},
                        str(save_iter))    

if __name__ == '__main__':
    rospy.init_node("mpc_test", anonymous=True)
    run_name = stream_tee.generate_timestamp()
    
    t = time.time()
    i = 0
    save_iter = 0
    rate = rospy.Rate(20) #hz
    term_mode = 'PTC'
    env = ENV(run_name,term_mode)
    env.reset()
    while not rospy.is_shutdown():
        done = env.done()
        if done==True:
            elapsed = time.time() - t
            print("Episode ", i, ' time = ', elapsed)
            i+=1
            print("Episode", i, " is started")
            env.reset()
            t = time.time()
        else:
            env.step()
        rate.sleep()

    
        
