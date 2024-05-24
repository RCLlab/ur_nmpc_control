#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <std_msgs/Int32.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosgraph_msgs/Clock.h>
#include "acados_solver_holder.cpp"
using namespace std;

float dist_v(Eigen::Vector3f v, Eigen::Vector3f w){
	return (v-w).norm();
}

double z_sh = 0.1;
Eigen::MatrixXf get_cpose(float theta_1, float theta_2, float theta_3, float theta_4, float theta_5, float theta_6){
Eigen::MatrixXf mat(3,8);
mat << 0, 0.06*sin(theta_1), (-0.425*cos(theta_1)*cos(theta_2))/2+0.14*sin(theta_1), -0.425*cos(theta_1)*cos(theta_2)+0.11*sin(theta_1), -0.425*cos(theta_1)*cos(theta_2)+(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1), -0.425*cos(theta_1)*cos(theta_2)+2*(-(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_1)*cos(theta_2)))/3+0.02*sin(theta_1), -(cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000+0.06*sin(theta_1), 0.10915*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2) + 0.0823*cos(theta_5)*sin(theta_1) + 0.39225*cos(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.39225*cos(theta_1)*cos(theta_2)*cos(theta_3)-0.05*sin(theta_1),
       0,-0.06*cos(theta_1), (-0.425*cos(theta_2)*sin(theta_1))/2-0.14*cos(theta_1), -0.425*cos(theta_2)*sin(theta_1)-0.11*cos(theta_1), -0.425*cos(theta_2)*sin(theta_1)+(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1), -0.425*cos(theta_2)*sin(theta_1)+2*(-(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-(-0.425*cos(theta_2)*sin(theta_1)))/3-0.02*cos(theta_1), -(sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2)))/4000-0.06*cos(theta_1), 0.39225*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*cos(theta_1)*cos(theta_5) - 0.425*cos(theta_2)*sin(theta_1) - 0.10915*cos(theta_1) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.09465*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.09465*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*cos(theta_2)*cos(theta_3)*sin(theta_1)+0.05*cos(theta_1),
       0, 0.0894+z_sh,       (0.0894 - 0.425*sin(theta_2))/2+z_sh,                    0.0894 - 0.425*sin(theta_2)+z_sh,                       0.0894 - 0.425*sin(theta_2)+(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh,                                            0.0894 - 0.425*sin(theta_2)+2*(0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)-(0.0894 - 0.425*sin(theta_2)))/3+z_sh,                                            0.0894 - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3)+z_sh,                                 0.09465*sin(theta_2 + theta_3)*sin(theta_4) - 0.425*sin(theta_2) - 0.39225*sin(theta_2 + theta_3) - sin(theta_5)*(0.0823*cos(theta_2 + theta_3)*sin(theta_4) + 0.0823*sin(theta_2 + theta_3)*cos(theta_4)) - 0.09465*cos(theta_2 + theta_3)*cos(theta_4) + 0.08945+z_sh;
	return mat;
}

// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 
    // Data Members 
    ros::Publisher chatter_pub;
    double human_sphere_predicted[56]={0}; 
    double goal[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
    double joint_position[6] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000};
  
    // Member Functions() 
    void change_obstacles_msg_predicted(const std_msgs::Float64MultiArray obstacle_data) { 
      for (int i=0; i<56; i++) human_sphere_predicted[i] = obstacle_data.data[i];
    }

    void change_states_msg(const std_msgs::Float64MultiArray::ConstPtr& msg) { 
       for (int i=0; i<6; i++) joint_position[i] = msg->data[i];
    }

    void SendVelocity(const std_msgs::Float64MultiArray joint_vel_values){
    	chatter_pub.publish(joint_vel_values);
	    return;
    }
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PTC");
  ros::NodeHandle n;
  ROS_INFO("Node Started");
  //--------------------------------
  GoalFollower my_follower;
  my_follower.chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/HighController/mpc_high_positions", 1);
  
  double read_goal[2][6] = { 2.619, -0.958, -1.22, -1.518, -1.588, 0.5,
                              0.0, -2.0, -1.22, -1.518, -1.588, 0.5};

  int horizon = 5;
  int row_index = 1;
  //------------------------------
  ros::Subscriber human_status = n.subscribe("/Obstacle/mpc_high_spheres", 1, &GoalFollower::change_obstacles_msg_predicted, &my_follower);
  ros::Subscriber joint_status = n.subscribe("/joint_states_high", 1, &GoalFollower::change_states_msg, &my_follower);

  my_NMPC_solver myMpcSolver=my_NMPC_solver(10,horizon);

  std_msgs::Float64MultiArray joint_vel_values;
  double cgoal[3];
  
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    double current_joint_position[6];
    double current_human_position_predicted[56]={0};
    double current_joint_goal[6];
    for (int i = 0; i < 6; ++i) current_joint_position[ i ] = my_follower.joint_position[ i ];
    for (int i = 0; i < 6; ++i) current_joint_goal[ i ] = read_goal[row_index][i];
    for (int i = 0; i < 56; ++i) current_human_position_predicted[ i ] = my_follower.human_sphere_predicted[ i ];

    Eigen::MatrixXf cgoal_mat = get_cpose(read_goal[row_index][0], read_goal[row_index][1], 
          read_goal[row_index][2], read_goal[row_index][3], read_goal[row_index][4], read_goal[row_index][5]);
    cgoal[0] = cgoal_mat.coeff(0, 7);
    cgoal[1] = cgoal_mat.coeff(1, 7);
    cgoal[2] = cgoal_mat.coeff(2, 7);

    double result[16]={0.0};
    int status=myMpcSolver.solve_my_mpc(current_joint_position, current_human_position_predicted, current_joint_goal, cgoal, result);
    if (status==4) for (int i=0; i<14; i++) result[i] = 0.0;
    ROS_INFO("KKT %f; Status %i",result[14], status);

    // Check if arrived
    float max_diff = 0;
    for (int i = 0; i < 6; i++) {
        if (abs(current_joint_position[i] - current_joint_goal[i]) > max_diff) {
            max_diff = abs(current_joint_position[i] - current_joint_goal[i]); 
        }
    }
    ROS_INFO("max_diff %f",max_diff);

    // prepare to send commands
    joint_vel_values.data.clear();
    for (int i = 0; i < 12; i++) joint_vel_values.data.push_back(result[i]);
    for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(current_joint_position[i]);
    for (int i = 0; i < 6; i++) joint_vel_values.data.push_back(current_joint_goal[i]);
    for (int i = 0; i < 3; i++) joint_vel_values.data.push_back(cgoal[i]);
    for (int i = 0; i < 4; i++) joint_vel_values.data.push_back(result[12+i]);
    joint_vel_values.data.push_back(max_diff);
    my_follower.SendVelocity(joint_vel_values);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

