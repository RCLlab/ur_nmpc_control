#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <sensor_msgs/JointState.h>

// Static human position k=12500,x+1.0,y+0.7+0.6
double y_temp = 0.6;
double point_array_temp[56] = {0.506607150807041,	0.94371799687768+y_temp,	0.494820526134201,	0.511,
                              0.549320132618770,	0.94455572448431+y_temp,	0.138556195523178,	0.601,
                              0.565921840279959,	1.09975054556363+y_temp,	0.311945749543228,	0.451,
                              0.457580797335886,	0.79197578394062+y_temp,	0.280283354502809,	0.451,
                              0.475030045037120,	1.12786843456287+y_temp,	0.430665583646954,	0.421,
                              0.359464574334591,	0.76755791684336+y_temp,	0.393964081471970,	0.421,
                              0.384138249794282,	1.15598632356212+y_temp,	0.549385417750679,	0.421,
                              0.261348351333296,	0.74314004974611+y_temp,	0.507644808441132,	0.421,
                              0.253646493641181,	1.09037601634713+y_temp,	0.628618388210253,	0.401,
                              0.179788775043484,	0.84293360405772+y_temp,	0.612525485774453,	0.401,
                              0.130678783643628,	1.00280482298653+y_temp,	0.716876603457698,	0.411,
                              0.083309998654304,	0.96330242695125+y_temp,	0.690034936267612,	0.411,
                              0.519994760519231,	0.94041703704492+y_temp,	0.333327933042604,	0.461,
                              0.569384119866634,	0.94762985826406+y_temp,	-0.0347318188968613,	0.441};


float point_array_temp[56];
float point_array[56];
double from_high[31] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10};
float sphere_radi[14]={0.511, 0.601, 0.451, 0.451, 0.421, 0.421, 0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441};

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

    for (int i = 0; i < 12; ++i) state_feedback[i] = state_feedback_temp[i];
    std_msgs::Float64MultiArray obstacle_data;
    obstacle_data.data.clear();
    for (int i = 0; i < 56; i++){
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

