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

    j_pelvis_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_l_position_controller/command",1);
    j_thigh1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_l_position_controller/command",1);
    j_thigh2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_l_position_controller/command",1);
    j_tibia_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tibia_l_position_controller/command",1);
    j_ankle1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_l_position_controller/command",1);
    j_ankle2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_l_position_controller/command",1);
    j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command",1);

    j_pelvis_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_r_position_controller/command",1);
    j_thigh1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_r_position_controller/command",1);
    j_thigh2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_r_position_controller/command",1);
    j_tibia_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tibia_r_position_controller/command",1);
    j_ankle1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_r_position_controller/command",1);
    j_ankle2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_r_position_controller/command",1);
    j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command",1);

    cmd_vel_subscriber_ = nh_.subscribe("/robotis_op/cmd_vel", 100, &GazeboWalkingNode::cmdVelCb, this);
    enable_walking_subscriber_ = nh_.subscribe("/robotis_op/enable_walking", 100, &GazeboWalkingNode::enableWalkCb, this);
    imu_subscriber_ = nh_.subscribe("/robotis_op/imu", 100, &GazeboWalkingNode::imuCb, this);
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

    double outValue[14];
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



void GazeboWalkingNode::imuCb(sensor_msgs::ImuConstPtr msg)
{
    walking_.fbGyroErr = msg->linear_acceleration.x*0.1;
    walking_.rlGyroErr = -msg->linear_acceleration.y*0.02;
}


void GazeboWalkingNode::dynamicReconfigureCb(robotis_op_gazebo::robotis_op_walkingConfig &config, uint32_t level)
{
    walking_.X_OFFSET=config.X_OFFSET;
    walking_.Y_OFFSET=config.Y_OFFSET;
    walking_.Z_OFFSET=config.Z_OFFSET;
    walking_.R_OFFSET=config.R_OFFSET;
    walking_.P_OFFSET=config.P_OFFSET;
    walking_.A_OFFSET=config.A_OFFSET;
    walking_.PERIOD_TIME=config.PERIOD_TIME;
    walking_.DSP_RATIO=config.DSP_RATIO;
    walking_.STEP_FB_RATIO=config.STEP_FB_RATIO;
    walking_.Z_MOVE_AMPLITUDE=config.Z_MOVE_AMPLITUDE;
    walking_.Y_SWAP_AMPLITUDE=config.Y_SWAP_AMPLITUDE;
    walking_.PELVIS_OFFSET=config.PELVIS_OFFSET;
    walking_.ARM_SWING_GAIN=config.ARM_SWING_GAIN;
    walking_.BALANCE_KNEE_GAIN=config.BALANCE_KNEE_GAIN;
    walking_.BALANCE_ANKLE_PITCH_GAIN=config.BALANCE_ANKLE_PITCH_GAIN;
    walking_.BALANCE_HIP_ROLL_GAIN=config.BALANCE_HIP_ROLL_GAIN;
    walking_.BALANCE_ANKLE_ROLL_GAIN=config.BALANCE_ANKLE_ROLL_GAIN;
    walking_.HIP_PITCH_OFFSET=config.HIP_PITCH_OFFSET;
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


    dynamic_reconfigure::Server<robotis_op_gazebo::robotis_op_walkingConfig> srv;
    dynamic_reconfigure::Server<robotis_op_gazebo::robotis_op_walkingConfig>::CallbackType cb;
    cb = boost::bind(&GazeboWalkingNode::dynamicReconfigureCb, &gazebo_walking_node, _1, _2);
    srv.setCallback(cb);


    ROS_INFO("Started walking");

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        gazebo_walking_node.Process();
        last_time = current_time;

    }

    return 0;
}

