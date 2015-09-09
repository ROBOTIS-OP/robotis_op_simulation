#ifndef GAZEBO_WALKING_NODE_H
#define GAZEBO_WALKING_NODE_H

#include <ros/ros.h>

#include <robotis_op_gazebo/gazebo_walking.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

namespace robotis_op {

class GazeboWalkingNode {
public:
    GazeboWalkingNode(ros::NodeHandle nh);
    ~GazeboWalkingNode();

    void Process();
    void enableWalkCb(std_msgs::BoolConstPtr enable);
    void cmdVelCb(const geometry_msgs::Twist::ConstPtr& msg);


protected:


private:

    GazeboWalking walking_;

    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber enable_walking_subscriber_;

    ros::Publisher j_pelvis_l_publisher_;
    ros::Publisher j_thigh1_l_publisher_;
    ros::Publisher j_thigh2_l_publisher_;
    ros::Publisher j_tibia_l_publisher_;
    ros::Publisher j_ankle1_l_publisher_;
    ros::Publisher j_ankle2_l_publisher_;
    ros::Publisher j_shoulder_l_publisher_;

    ros::Publisher j_pelvis_r_publisher_;
    ros::Publisher j_thigh1_r_publisher_;
    ros::Publisher j_thigh2_r_publisher_;
    ros::Publisher j_tibia_r_publisher_;
    ros::Publisher j_ankle1_r_publisher_;
    ros::Publisher j_ankle2_r_publisher_;
    ros::Publisher j_shoulder_r_publisher_;



};

}
#endif //GAZEBO_WALKING_NODE_H
