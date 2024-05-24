#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sensor_msgs/JointState.h>
double a = 10.2;
double b = 10.2;
double c = 1.2;
double point_array_temp[56]={0.4479+b,    1.0382+a,   0.4868+c, 0.511,
                            0.5414+b,    1.0771+a,    0.1415+c, 0.601,
                            0.5291+b,    1.2252+a,    0.3120+c, 0.451,
                            0.4541+b,    0.9102+a,    0.2589+c, 0.451,
                            0.4507+b,    1.3385+a,    0.3764+c, 0.421,
                            0.3177+b,    0.8561+a,    0.2988+c, 0.421,
                            0.3722+b,    1.4519+a,    0.4409+c, 0.421,
                            0.1812+b,    0.8020+a,    0.3387+c, 0.421,
                            0.2454+b,    1.4003+a,    0.5351+c, 0.401,
                            0.0557+b,    0.8889+a,    0.4043+c, 0.401,
                            0.1536+b,    1.3396+a,    0.6639+c, 0.411,
                            -0.0633+b,    0.9652+a,    0.5038+c, 0.411,
                            0.4991+b,    1.0584+a,    0.3330+c, 0.461,
                            0.5696+b,    1.0902+a,   -0.0302+c, 0.441}; 
// float point_array_temp_high[560];
float point_array[58];
// float point_array_high[560];
double from_high[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10};
float sphere_radi[14] = {0.511, 0.601, 0.451, 0.451, 0.421, 0.421, 0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441};

void chatterCallback(const std_msgs::Float64MultiArray msg)
{
  for (int i = 0; i<14; i++){
    point_array_temp[i*4] = msg.data[i*3];
    point_array_temp[i*4+1] = msg.data[i*3+1];
    point_array_temp[i*4+2] = msg.data[i*3+2];
    point_array_temp[i*4+3] = sphere_radi[i];
  }
  point_array_temp[56] = msg.data[463];
  point_array_temp[57] = msg.data[464];
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
      point_array[i*4] = point_array_temp[i*4]; // x offset
      point_array[i*4+1] = point_array_temp[i*4+1]; // y offset
      point_array[i*4+2] = point_array_temp[i*4+2]-1.2; // z offset
      point_array[i*4+3] = point_array_temp[i*4+3];
    }
    
    point_array[56]=point_array_temp[56];
    point_array[57]=point_array_temp[57];

    for (int i = 0; i < 12; ++i) state_feedback[i] = state_feedback_temp[i];
    std_msgs::Float64MultiArray obstacle_data;
    obstacle_data.data.clear();
    for (int i = 0; i < 58; i++){
      obstacle_data.data.push_back(point_array[i]);
    } 
    chatter_low.publish(obstacle_data);
    std_msgs::Float64MultiArray obstacle_data_high;
    obstacle_data_high.data.clear();
    for (int i = 0; i < 56; i++){
      obstacle_data_high.data.push_back(point_array[i]);
    } 
    chatter_high.publish(obstacle_data_high);

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

