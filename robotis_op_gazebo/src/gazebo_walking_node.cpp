#include <robotis_op_gazebo/gazebo_walking_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>

#include <robotis_op_gazebo/math/Matrix.h>
#define MX28_1024

namespace robotis_op {
using namespace Robot;



GazeboWalkingNode::GazeboWalkingNode(ros::NodeHandle nh)
    : nh_(nh)
    , walking_(nh)
{

    j_pelvis_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_pelvis_l_position_controller/command",1);
    j_thigh1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_thigh1_l_position_controller/command",1);
    j_thigh2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_thigh2_l_position_controller/command",1);
    j_tibia_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_tibia_l_position_controller/command",1);
    j_ankle1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_ankle1_l_position_controller/command",1);
    j_ankle2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_ankle2_l_position_controller/command",1);
    j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_shoulder_l_position_controller/command",1);




    j_pelvis_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_pelvis_r_position_controller/command",1);
    j_thigh1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_thigh1_r_position_controller/command",1);
    j_thigh2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_thigh2_r_position_controller/command",1);
    j_tibia_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_tibia_r_position_controller/command",1);
    j_ankle1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_ankle1_r_position_controller/command",1);
    j_ankle2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_ankle2_r_position_controller/command",1);
    j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/darwin/j_shoulder_l_position_controller/command",1);

    cmd_vel_subscriber_ = nh_.subscribe("cmd_vel", 100, &GazeboWalkingNode::cmdVelCb, this);
    enable_walking_subscriber_ = nh_.subscribe("enable_walking", 100, &GazeboWalkingNode::enableWalkCb, this);


}

GazeboWalkingNode::~GazeboWalkingNode()
{
}




void GazeboWalkingNode::Process()
{

    std_msgs::Float64 j_pelvis_l_msg;
    std_msgs::Float64 j_thigh1_l_msg;
    std_msgs::Float64 j_thigh2_l_msg;
    std_msgs::Float64 j_tibia_l_msg;
    std_msgs::Float64 j_ankle1_l_msg;
    std_msgs::Float64 j_ankle2_l_msg;
    std_msgs::Float64 j_shoulder_l_msg;
    std_msgs::Float64 j_pelvis_r_msg;
    std_msgs::Float64 j_thigh1_r_msg;
    std_msgs::Float64 j_thigh2_r_msg;
    std_msgs::Float64 j_tibia_r_msg;
    std_msgs::Float64 j_ankle1_r_msg;
    std_msgs::Float64 j_ankle2_r_msg;
    std_msgs::Float64 j_shoulder_r_msg;

    double outValue[14]; //todo
    walking_.Process(&outValue[0]);


    j_pelvis_r_msg.data = outValue[0];
    j_pelvis_l_msg.data = outValue[6];
    j_thigh1_r_msg.data = outValue[1];
    j_thigh1_l_msg.data = outValue[7];
    j_thigh2_r_msg.data = outValue[2];
    j_thigh2_l_msg.data = outValue[8];
    j_tibia_r_msg.data = outValue[3];
    j_tibia_l_msg.data = outValue[9];
    j_ankle1_r_msg.data = outValue[4];
    j_ankle1_l_msg.data = outValue[10];
    j_ankle2_r_msg.data = outValue[5];
    j_ankle2_l_msg.data = outValue[11];
    j_shoulder_r_msg.data = outValue[12];
    j_shoulder_l_msg.data = outValue[13];


    j_pelvis_l_publisher_.publish(j_pelvis_l_msg);
    j_thigh1_l_publisher_.publish(j_thigh1_l_msg);
    j_thigh2_l_publisher_.publish(j_thigh2_l_msg);
    j_tibia_l_publisher_.publish(j_tibia_l_msg);
    j_ankle1_l_publisher_.publish(j_ankle1_l_msg);
    j_ankle2_l_publisher_.publish(j_ankle2_l_msg);
    j_shoulder_l_publisher_.publish(j_shoulder_l_msg);

    j_pelvis_r_publisher_.publish(j_pelvis_r_msg);
    j_thigh1_r_publisher_.publish(j_thigh1_r_msg);
    j_thigh2_r_publisher_.publish(j_thigh2_r_msg);
    j_tibia_r_publisher_.publish(j_tibia_r_msg);
    j_ankle1_r_publisher_.publish(j_ankle1_r_msg);
    j_ankle2_r_publisher_.publish(j_ankle2_r_msg);
    j_shoulder_r_publisher_.publish(j_shoulder_r_msg);

}

void GazeboWalkingNode::cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg)
{
    double period = walking_.PERIOD_TIME;
    walking_.X_MOVE_AMPLITUDE=(msg->linear.x/period*1000.0);
    walking_.Y_MOVE_AMPLITUDE=(msg->linear.y/period*1000.0);
    // compute the angular motion parameters to achieve the desired angular speed
    walking_.A_MOVE_AMPLITUDE=(msg->angular.z/period*180.0)/(2.0*3.14159);

}


void GazeboWalkingNode::enableWalkCb(std_msgs::BoolConstPtr enable)
{
    if(enable->data)
    {
        if(!walking_.IsRunning())
        {
            walking_.Start();
        }
    }
    else
    {
        if(walking_.IsRunning())
        {
            walking_.Stop();
        }
    }
}

}



int main(int argc, char **argv)
{

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    double control_rate;
    nh.param("robotis_op_walking/control_rate", control_rate, 125.0);
    control_rate = 125.0;

    GazeboWalkingNode gazebo_walking_node(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);
    ROS_INFO("Starting walking");
    //gazebo_walking.Start();

    ROS_INFO("Started walking");


    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        gazebo_walking_node.Process();
        // gazebo_walking.update(current_time, elapsed_time);
        last_time = current_time;
    }

    return 0;
}

