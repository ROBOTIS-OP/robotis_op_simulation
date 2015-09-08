#include <robotis_op_gazebo/gazebo_walking.h>
using namespace robotis_op;

int main(int argc, char **argv)
{

  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle nh;
  double control_rate;
  nh.param("robotis_op_walking/control_rate", control_rate, 125.0);
  control_rate = 125.0;

  gazebo_walking gazebo_walking(nh, ros::NodeHandle("~"));

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Time last_time = ros::Time::now();
  ros::Rate rate(control_rate);
  ROS_INFO("Starting walking");
  gazebo_walking.Start();

  ROS_INFO("Started walking");


  while (ros::ok())
  {
      rate.sleep();
      ros::Time current_time = ros::Time::now();
      ros::Duration elapsed_time = current_time - last_time;
      gazebo_walking.Process();
     // gazebo_walking.update(current_time, elapsed_time);
      last_time = current_time;
  }

  return 0;
}

