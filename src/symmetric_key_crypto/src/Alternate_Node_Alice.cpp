#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

class PubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  ros::Rate m_Rate;
  public:
  PubSubHandler(const ros::Publisher& _publisher,const ros::Rate _rate)
  :m_Publisher(_publisher),m_Rate(_rate)
  {
    std_msgs::String message;
    std::stringstream stream;
    stream << "Let's start a conversation";
    message.data = stream.str();
    m_Publisher.publish(message);
  }
  
  void MessageReceived(const std_msgs::String::ConstPtr& _message)
  {
    ROS_INFO("Alice ->  Bob said: [%s]",_message -> data.c_str());
    m_Rate.sleep();
    std_msgs::String _response_message;
    std::stringstream _stream;
    _stream << "Alice ping";
    _response_message.data = _stream.str();
    m_Publisher.publish(_response_message);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"alternate_node_alice");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(1);
  ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("message_from_alice",10,true);
  PubSubHandler pubsubHandler = {publisher,rate};
  ros::Subscriber subscriber = nodeHandle.subscribe("message_for_alice",1000,&PubSubHandler::MessageReceived,&pubsubHandler);
  ros::spin();
  return EXIT_SUCCESS;
}
