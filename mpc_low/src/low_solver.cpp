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
#include <visualization_msgs/Marker.h>

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

Eigen::MatrixXf get_velocity(float theta_1, float theta_2, float theta_3, float theta_4, float theta_5, float theta_6, 
                             float u_1, float u_2, float u_3, float u_4, float u_5, float u_6){
	Eigen::MatrixXf mat(21,1);
  mat << 0.06*u_1*cos(theta_1),
        0.06*u_1*sin(theta_1),
        0,
        u_1*(0.14*cos(theta_1) + 0.2125*cos(theta_2)*sin(theta_1)) + 0.2125*u_2*cos(theta_1)*sin(theta_2),
        u_1*(0.14*sin(theta_1) - 0.2125*cos(theta_1)*cos(theta_2)) + 0.2125*u_2*sin(theta_1)*sin(theta_2),
        -0.2125*u_2*cos(theta_2),
        u_1*(0.11*cos(theta_1) + 0.425*cos(theta_2)*sin(theta_1)) + 0.425*u_2*cos(theta_1)*sin(theta_2),
        u_1*(0.11*sin(theta_1) - 0.425*cos(theta_1)*cos(theta_2)) + 0.425*u_2*sin(theta_1)*sin(theta_2),
        -0.425*u_2*cos(theta_2),
        0.02*u_1*cos(theta_1) + 0.13075*u_1*cos(theta_2 + theta_3)*sin(theta_1) + 0.13075*u_2*sin(theta_2 + theta_3)*cos(theta_1) + 0.13075*u_3*sin(theta_2 + theta_3)*cos(theta_1) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2),
        0.02*u_1*sin(theta_1) - 0.13075*u_1*cos(theta_2 + theta_3)*cos(theta_1) + 0.13075*u_2*sin(theta_2 + theta_3)*sin(theta_1) + 0.13075*u_3*sin(theta_2 + theta_3)*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.425*u_2*sin(theta_1)*sin(theta_2),
        - 1.0*u_2*(0.13075*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.13075*u_3*cos(theta_2 + theta_3),
        0.02*u_1*cos(theta_1) + 0.2615*u_1*cos(theta_2 + theta_3)*sin(theta_1) + 0.2615*u_2*sin(theta_2 + theta_3)*cos(theta_1) + 0.2615*u_3*sin(theta_2 + theta_3)*cos(theta_1) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2),
        0.02*u_1*sin(theta_1) - 0.2615*u_1*cos(theta_2 + theta_3)*cos(theta_1) + 0.2615*u_2*sin(theta_2 + theta_3)*sin(theta_1) + 0.2615*u_3*sin(theta_2 + theta_3)*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.425*u_2*sin(theta_1)*sin(theta_2),
        - 1.0*u_2*(0.2615*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.2615*u_3*cos(theta_2 + theta_3),
        u_1*(0.06*cos(theta_1) + 0.00025*sin(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2))) + 0.00025*u_2*cos(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*cos(theta_1),
        u_1*(0.06*sin(theta_1) - 0.00025*cos(theta_1)*(1569.0*cos(theta_2 + theta_3) + 1700.0*cos(theta_2))) + 0.00025*u_2*sin(theta_1)*(1569.0*sin(theta_2 + theta_3) + 1700.0*sin(theta_2)) + 0.39225*u_3*sin(theta_2 + theta_3)*sin(theta_1),
        - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2)) - 0.39225*u_3*cos(theta_2 + theta_3),
        0.05915*u_1*cos(theta_1) + 0.0823*u_1*cos(theta_1)*cos(theta_5) + 0.425*u_1*cos(theta_2)*sin(theta_1) + 0.425*u_2*cos(theta_1)*sin(theta_2) - 0.0823*u_5*sin(theta_1)*sin(theta_5) + 0.09465*u_2*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_3*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_4*cos(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) - 0.09465*u_1*cos(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_1*sin(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.09465*u_2*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) - 0.09465*u_3*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) - 0.09465*u_4*sin(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.39225*u_1*cos(theta_2)*cos(theta_3)*sin(theta_1) + 0.39225*u_2*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*u_2*cos(theta_1)*cos(theta_3)*sin(theta_2) + 0.39225*u_3*cos(theta_1)*cos(theta_2)*sin(theta_3) + 0.39225*u_3*cos(theta_1)*cos(theta_3)*sin(theta_2) - 0.39225*u_1*sin(theta_1)*sin(theta_2)*sin(theta_3) - 0.0823*u_5*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*cos(theta_5) + 0.0823*u_1*cos(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_2*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.0823*u_3*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) + 0.0823*u_4*sin(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5),
        0.05915*u_1*sin(theta_1) - 0.425*u_1*cos(theta_1)*cos(theta_2) + 0.0823*u_1*cos(theta_5)*sin(theta_1) + 0.0823*u_5*cos(theta_1)*sin(theta_5) + 0.425*u_2*sin(theta_1)*sin(theta_2) + 0.09465*u_1*cos(theta_2 + theta_3)*cos(theta_1)*sin(theta_4) + 0.09465*u_1*sin(theta_2 + theta_3)*cos(theta_1)*cos(theta_4) + 0.09465*u_2*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.09465*u_3*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) + 0.09465*u_4*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_1) - 0.39225*u_1*cos(theta_1)*cos(theta_2)*cos(theta_3) - 0.09465*u_2*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_3*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) - 0.09465*u_4*sin(theta_2 + theta_3)*sin(theta_1)*sin(theta_4) + 0.39225*u_1*cos(theta_1)*sin(theta_2)*sin(theta_3) + 0.39225*u_2*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*u_2*cos(theta_3)*sin(theta_1)*sin(theta_2) + 0.39225*u_3*cos(theta_2)*sin(theta_1)*sin(theta_3) + 0.39225*u_3*cos(theta_3)*sin(theta_1)*sin(theta_2) - 0.0823*u_1*cos(theta_2 + theta_3 + theta_4)*cos(theta_1)*sin(theta_5) - 0.0823*u_5*cos(theta_2 + theta_3 + theta_4)*cos(theta_5)*sin(theta_1) + 0.0823*u_2*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_3*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5) + 0.0823*u_4*sin(theta_2 + theta_3 + theta_4)*sin(theta_1)*sin(theta_5),
        u_4*(0.09465*sin(theta_2 + theta_3 + theta_4) - 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_3*(0.39225*cos(theta_2 + theta_3) - 0.09465*sin(theta_2 + theta_3 + theta_4) + 0.0823*cos(theta_2 + theta_3 + theta_4)*sin(theta_5)) - 1.0*u_2*(0.39225*cos(theta_2 + theta_3) + 0.425*cos(theta_2) - 0.09465*cos(theta_2 + theta_3)*sin(theta_4) - 0.09465*sin(theta_2 + theta_3)*cos(theta_4) + 0.0823*cos(theta_2 + theta_3)*cos(theta_4)*sin(theta_5) - 0.0823*sin(theta_2 + theta_3)*sin(theta_4)*sin(theta_5)) - 0.0823*u_5*sin(theta_2 + theta_3 + theta_4)*cos(theta_5);
  return mat;
}

// Introduce class to make safer goal change
class GoalFollower 
{ 
    // Access specifier 
    public: 

    // Data Members 
    ros::Publisher info_pub;
    double robot_spheres[7] = {0.15, 0.15, 0.15, 0.08, 0.08, 0.12, 0.1};
    
    double human_sphere[56]= {10.0517,   0.5220,   1.0895,   0.1500,
                              10.0658,   0.4526,   0.8624,   0.2500,
                              10.0844,   0.7044,   0.9207,   0.1500,
                              10.2083,   0.3075,   1.0208,   0.1500,
                              10.0556,   0.6289,   0.7595,   0.1500,
                              10.2024,   0.2732,   0.8478,   0.1500,
                              10.0267,   0.5535,   0.5983,   0.1500,
                              10.1965,   0.2389,   0.6749,   0.1500,
                             -10.0208,   0.3964,   0.5857,   0.1000,
                              10.0546,   0.2951,   0.6132,   0.1000,
                             -10.1062,   0.2444,   0.5897,   0.1300,
                             -10.0998,   0.3062,   0.5387,   0.1300,
                              10.1908,   0.5290,   1.0016,   0.2000,
                              10.2106,   0.4602,   0.6915,   0.2500};

    double goal_queue[30] = {0.0000, -1.57, 0.0000, -1.57, 0.0000, 0.0000, 
                              0.0000, -1.57, 0.0000, -1.57, 0.0000, 0.0000, 
                              0.0000, -1.57, 0.0000, -1.57, 0.0000, 0.0000, 
                              0.0000, -1.57, 0.0000, -1.57, 0.0000, 0.0000,
                              0.0000, -1.57, 0.0000, -1.57, 0.0000, 0.0000};
                              
    double goal[6] = {0};
    double jp[6] = {0};
    double from_high[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10};

    // Member Functions() 
    void change_obstacles_msg(const std_msgs::Float64MultiArray obstacle_data){ 
      for (int i=0; i<56; i++) human_sphere[i] = obstacle_data.data[i];
    }

    void change_goal_msg(const std_msgs::Float64MultiArray joint_pose_values) { 
      for (int i=0; i<32; i++) from_high[i]= joint_pose_values.data[i];

      for (int j=0; j<5; j++) {
        for (int i=0; i<6; i++){
          goal_queue[j*6+i] = joint_pose_values.data[i]*(0.050*(j+5)) + joint_pose_values.data[i+12]; 
        }
      }
    }

    void change_states_msg(const std_msgs::Float64MultiArray::ConstPtr& msg){
      for (int i=0; i<6; i++) jp[i] = msg->data[i];
    }

    void SendInfo(const std_msgs::Float64MultiArray data){
    	info_pub.publish(data);
	    return;
    }
}; 


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "joint_controller_low");
  ros::NodeHandle n;
  ROS_INFO("Node Started");

  GoalFollower upd;
  upd.info_pub = n.advertise<std_msgs::Float64MultiArray>("/info", 1);
  ros::Subscriber joint_status = n.subscribe("/joint_states_low", 1, &GoalFollower::change_states_msg, &upd);
  ros::Subscriber joint_goal = n.subscribe("/HighController/mpc_high_positions", 1, &GoalFollower::change_goal_msg, &upd);
  ros::Subscriber human_status = n.subscribe("/Obstacle/mpc_low_spheres", 1, &GoalFollower::change_obstacles_msg, &upd);
  
  double smallest_dist;
  double local_val;
  double ctv_linear[7] = {0};
  double min_dist[7] = {10000, 10000, 10000, 10000, 10000, 10000, 10000};
  
  my_NMPC_solver myMpcSolver=my_NMPC_solver(5);
  
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    printf("max diff = %f\n",upd.from_high[31]);
    double cgoal[3];
    double goal[6];
    double tracking_goal[30];

    for (int i = 0; i < 6; ++i) goal[i] = upd.goal_queue[i];
    for (int i = 0; i < 3; ++i) cgoal[i] = upd.from_high[24+i];
    for (int i = 0; i < 30; ++i) tracking_goal[i] = upd.goal_queue[i];

    //******************* get_min_dist **********************
	  smallest_dist = 10000;

    // for 7 points on UR5:
    min_dist[0] = 10000; min_dist[1] = 10000; min_dist[2] = 10000; 
    min_dist[3] = 10000; min_dist[4] = 10000; min_dist[5] = 10000; min_dist[6] = 10000; 

    double spheres_dist[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::MatrixXf mat2 = get_cpose(upd.jp[0], upd.jp[1],upd.jp[2], upd.jp[3],upd.jp[4], upd.jp[5]);
   
    for (int j = 0; j < 7; j++) {
      Eigen::Vector3f w;
      w = mat2.col(j+1).transpose();
      for (int k = 0; k < 14; k++) {
        Eigen::Vector3f p(upd.human_sphere[k*4+0], upd.human_sphere[k*4+1], upd.human_sphere[k*4+2]);
        local_val = dist_v(w, p) - upd.robot_spheres[j] - upd.human_sphere[k*4+3];
        if (local_val < min_dist[j]) {
          min_dist[j] = local_val;
          spheres_dist[j] = upd.robot_spheres[j] + upd.human_sphere[k*4+3];
        }
      }
	    if (smallest_dist > min_dist[j]) smallest_dist = min_dist[j];
	  }

    ROS_INFO("SD %f ", smallest_dist);
    double result[16] = {0.0};
    double trajectory[66] = {0.0};
    if (smallest_dist >= 0.00 && upd.from_high[0]!=0.0) {
      clock_t begin_mpc = clock();
      int status = myMpcSolver.solve_my_mpc(upd.jp, goal, upd.human_sphere, cgoal, tracking_goal, result, trajectory);   
      if (status > 0) {
            ROS_INFO("Destroying solver object");
            myMpcSolver.reset_solver();
            myMpcSolver=my_NMPC_solver(5);
            ROS_INFO("Solver recreated");
      }
      if (status==4) for (int i=0; i<12; i++) result[i] = 0.0;
      } else for (int i=0; i<16; i++) result[i] = 0;

    //******************* get_min_velocity **********************
    double max_vell[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::MatrixXf vell_mat = get_velocity(upd.jp[0], upd.jp[1], upd.jp[2], upd.jp[3],upd.jp[4], upd.jp[5],
                                            result[0], result[1], result[2],
                                            result[3], result[4], result[5]);
    double max_linear_vell = 0;
    double temp_linear_vell = 0;
    
    for (int k=0; k<7; k++) {
      temp_linear_vell = sqrt(vell_mat.coeff(k*3 + 0,0)*vell_mat.coeff(k*3 + 0,0) + vell_mat.coeff(k*3 + 1,0)*vell_mat.coeff(k*3 + 1,0) + vell_mat.coeff(k*3 + 2,0)*vell_mat.coeff(k*3 + 2,0));
      max_vell[k] = temp_linear_vell;
      if (max_linear_vell < temp_linear_vell) max_linear_vell = temp_linear_vell;
    }

    //*********************** Apply control ********************************
	  double lin_vell_limit_arr[7] = {10, 10, 10, 10, 10, 10, 10};
    double lin_vell_scale = 10;

    double alpha= 0.15;
    double sqrt_temp_value = 0.00;
    for (int i=0; i<7; i++) {
      sqrt_temp_value = (min_dist[i]+spheres_dist[i])*(min_dist[i]+spheres_dist[i])-spheres_dist[i]*spheres_dist[i];
      if (sqrt_temp_value<0) lin_vell_limit_arr[i] = 0.00000000000000;
      else lin_vell_limit_arr[i] = alpha*sqrt(sqrt_temp_value);
      double temp_scale = (lin_vell_limit_arr[i]/max_vell[i]);
      if (lin_vell_scale>temp_scale) lin_vell_scale = temp_scale;
    }

    if (lin_vell_scale<1.0) for (int i = 0; i < 6; i++) result[i] = result[i]*lin_vell_scale;
    
    vell_mat = get_velocity(upd.jp[0], upd.jp[1], upd.jp[2], upd.jp[3],upd.jp[4], upd.jp[5],
                            result[0], result[1], result[2],
                            result[3], result[4], result[5]);
          
    for (int k=0; k<7; k++) {
      temp_linear_vell = sqrt(vell_mat.coeff(k*3 + 0,0)*vell_mat.coeff(k*3 + 0,0) + vell_mat.coeff(k*3 + 1,0)*vell_mat.coeff(k*3 + 1,0) + vell_mat.coeff(k*3 + 2,0)*vell_mat.coeff(k*3 + 2,0));
      ctv_linear[k] = temp_linear_vell;
    }

    ROS_INFO("Max lin vell %.3f, Vel Index %.3f,SD %.3f", max_linear_vell, lin_vell_scale, smallest_dist);

    // data sending
    std_msgs::Float64MultiArray whole_data;
    whole_data.data.clear();
    for (int i = 0; i < 6; i++) whole_data.data.push_back(upd.jp[i]);
    for (int i = 0; i < 14; i++) for(int j = 0; j < 3; j++) whole_data.data.push_back(upd.human_sphere[i*4+j]);
    for (int i = 0; i < 16; i++) whole_data.data.push_back(result[i]);
    for (int i = 0; i < 6; i++) whole_data.data.push_back(upd.goal[i]);
    for (int i = 0; i < 32; i++) whole_data.data.push_back(upd.from_high[i]);
    for (int i = 0; i < 7; i++) whole_data.data.push_back(lin_vell_limit_arr[i]);
    for (int i = 0; i < 7; i++) whole_data.data.push_back(ctv_linear[i]);
    upd.SendInfo(whole_data);
    
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

