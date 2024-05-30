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
        self.init_poses = [2.6, -1.2, -1.22, -1.518, -1.588, 0.5]
        # self.init_poses = [3.0, -2.0, -1.22, -1.518, -1.588, 0.5]
        # self.init_poses = [2.9, -1.2, -1.22, -1.518, -1.588, 0.5]
        self.term_mode = term_mode
        self.forcestop = 0
        self.first = 0
        self.i = 0
        self.init_log_variables()
        self.hello_str = [-1,0]
        self.max_diff = 10
        self.t_total = time.time()
        
    def callback(self, data):
        self.observation = data.data[0:112]

    def done(self):
        # Check if arrived
        arrive = False
        if self.max_diff<0.02 or self.forcestop>400:
            print("-----Arrived------", self.max_diff)
            arrive = True

        return arrive

    def step(self):
        self.forcestop+=1
        
        # Send velocity to the robot
        vel = [self.observation[48],self.observation[49],self.observation[50],self.observation[51],self.observation[52],self.observation[53]]
        hello_str = "speedj(["+str(vel[0])+","+str(vel[1])+","+str(vel[2])+","+str(vel[3])+","+str(vel[4])+","+str(vel[5])+"],"+"5.0"+",0.05)" 
        self.pub.publish(hello_str)
        # Collect all necessary variables
        self.joint_poses.append(self.observation[0:6])
        self.human_poses.append(self.observation[6:48])
        self.low_mpc_solver.append(self.observation[48:64])
        self.smallest_dist.append(self.observation[64])
        self.lin_vel_scale.append(self.observation[65])
        self.from_high_controller.append(self.observation[66:98])
        self.ctv_linear.append(self.observation[98:105])
        self.lin_vel_limit.append(self.observation[105:112])
        self.max_diff = self.observation[97]
    
    def reset(self): 
        # Sends the robot and human to initial poses
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
        # Initialization of all variables
        self.observation = [1]*112
        self.joint_poses= []
        self.human_poses= []
        self.low_mpc_solver= []
        self.smallest_dist= []
        self.lin_vel_scale= []
        self.from_high_controller= []
        self.ctv_linear= []
        self.lin_vel_limit= []
        self.max_diff = 10

    
    def save_log(self,save_iter):
        # Save all variables in mat file
        rec_dir = '/home/robot/workspaces/Big_Data/'
        os.chdir(rec_dir)
        write_mat('StabilityTests/' + self.term_mode +self.run_name,
                        {
                        'joint_positions': self.joint_poses,
                        'human_poses':self.human_poses,
                        'low_mpc_solver': self.low_mpc_solver,
                        'smallest_dist': self.smallest_dist,
                        'lin_vel_scale':self.lin_vel_scale,
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
    term_mode = 'SQP10/STC/'
    env = ENV(run_name,term_mode)
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

    
        
