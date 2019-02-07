#include "ros/ros.h"
#include "std_msgs/String.h"

void MessageReceived(const std_msgs::String::ConstPtr& _message)
{
  ROS_INFO("Subscribed message [%s]",_message -> data.c_str());
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"node_bob");
  ros::NodeHandle nodeHandle;
  ros::Subscriber subscriber = nodeHandle.subscribe("message_for_bob",1000,MessageReceived);
  ros::spin();
  return 0;
}
