#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

class PubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  public:
  PubSubHandler(const ros::Publisher& _publisher)
  :m_Publisher(_publisher){}
  
  void MessageReceived(const std_msgs::String::ConstPtr& _message)
  {
    ROS_INFO("Bob -> Alice said: [%s]",_message -> data.c_str());
    std_msgs::String _response_message;
    std::stringstream _stream;
    _stream << "Bob Ping";
    _response_message.data = _stream.str();
    m_Publisher.publish(_response_message);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"alternate_node_bob");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<std_msgs::String>("message_for_alice",1000);
  PubSubHandler pubsubHandler = publisher;

  ros::Subscriber subscriber = nodeHandle.subscribe("message_from_alice",1000,&PubSubHandler::MessageReceived,&pubsubHandler);
  ros::spin();
  return EXIT_SUCCESS;
}


