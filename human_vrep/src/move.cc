#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sensor_msgs/JointState.h>

double x_temp = 0.0;
double y_temp = 0.7;
double z_temp = -1.2;
double point_array_temp[56] = {0.4788+x_temp,   0.2294+y_temp,	1.6848+z_temp, 0.511,
                              0.4277+x_temp,	 0.1565+y_temp,	1.3404+z_temp, 0.601,
                              0.3693+x_temp,	 0.3098+y_temp,	1.4987+z_temp, 0.451,
                              0.5571+x_temp,	 0.0405+y_temp,	1.4860+z_temp, 0.451,
                              0.4197+x_temp,	 0.3990+y_temp,	1.6112+z_temp, 0.421,
                              0.6811+x_temp,	 0.0235+y_temp,	1.5726+z_temp, 0.421,
                              0.4701+x_temp,	 0.4882+y_temp,	1.7236+z_temp, 0.421,
                              0.8050+x_temp,	 0.0065+y_temp,	1.6592+z_temp, 0.421,
                              0.5925+x_temp,	 0.4673+y_temp,	1.8340+z_temp, 0.401,
                              0.8441+x_temp,	 0.1071+y_temp,	1.7855+z_temp, 0.401,
                              0.7306+x_temp,	 0.4507+y_temp,	1.9416+z_temp, 0.411,
                              0.8895+x_temp,	 0.2101+y_temp,	1.9212+z_temp, 0.411,
                              0.4575+x_temp,	 0.1695+y_temp,	1.5347+z_temp, 0.461,
                              0.4076+x_temp,	 0.1501+y_temp,	1.1672+z_temp, 0.441}; 

double point_array[56];
double sphere_radi[14] = {0.511, 0.601, 0.451, 0.451, 0.421, 0.421, 0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441};

void chatterCallback(const std_msgs::Float64MultiArray msg)
{
  for (int i = 0; i<14; i++){
    point_array_temp[i*4] = msg.data[i*3];
    point_array_temp[i*4+1] = msg.data[i*3+1];
    point_array_temp[i*4+2] = msg.data[i*3+2];
    point_array_temp[i*4+3] = sphere_radi[i];
  }
}
double state_feedback_temp[12];
double state_feedback[12];
void feedbackCB(const sensor_msgs::JointState msg) 
{
  for (int i = 0; i < 6; ++i) 
  {
    state_feedback_temp[i] = msg.position[i];
    state_feedback_temp[i+6] = msg.velocity[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "human_control");
  ros::NodeHandle nodeHandle("~");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<std_msgs::Float64MultiArray>("/Obstacle/human_spheres", 1, chatterCallback);
  ros::Subscriber arm_sub = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, feedbackCB);
  ros:: Publisher chatter_low = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_low_spheres", 1);
  ros:: Publisher chatter_high = n.advertise<std_msgs::Float64MultiArray>("/Obstacle/mpc_high_spheres", 1);
  ros:: Publisher states_low = n.advertise<std_msgs::Float64MultiArray>("/joint_states_low", 1);
  ros:: Publisher states_high = n.advertise<std_msgs::Float64MultiArray>("/joint_states_high", 1);

  while (ros::ok())
  {
    for (int i = 0; i<14; i++) {
      point_array[i*4] = point_array_temp[i*4]; 
      point_array[i*4+1] = point_array_temp[i*4+1]; 
      point_array[i*4+2] = point_array_temp[i*4+2]; 
      point_array[i*4+3] = point_array_temp[i*4+3];
    }
    std_msgs::Float64MultiArray obstacle_data;
    obstacle_data.data.clear();
    for (int i = 0; i < 56; i++)obstacle_data.data.push_back(point_array[i]);
    chatter_low.publish(obstacle_data);
    chatter_high.publish(obstacle_data);

    for (int i = 0; i < 12; ++i) state_feedback[i] = state_feedback_temp[i];
    std_msgs::Float64MultiArray state_data;
    state_data.data.clear();
    for (int i = 0; i < 12; i++) state_data.data.push_back(state_feedback[i]);
    states_low.publish(state_data);
    states_high.publish(state_data);

    ros::spinOnce();
  }
  ros::spin();
  return 0;
}

