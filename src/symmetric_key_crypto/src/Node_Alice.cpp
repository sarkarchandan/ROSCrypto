#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"node_alice");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("message_for_bob",1000);
  ros::Rate loopRate = 1;
  while(ros::ok())
  {
    std_msgs::String message;
    std::stringstream stream;
    stream << "Cipher message for bob";
    message.data = stream.str();
    ROS_INFO("Published message: [%s]",message.data.c_str());
    publisher.publish(message);
    ros::spinOnce();
    loopRate.sleep();
  }
  return EXIT_SUCCESS;
}
