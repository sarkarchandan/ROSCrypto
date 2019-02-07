#include "ros/ros.h"
#include "symmetric_key_crypto/cipher_array.h"
#include "symmetric_key_crypto/VectorHandler.h"

class PubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  public:
  PubSubHandler(const ros::Publisher& _publisher)
  :m_Publisher(_publisher){}
  
  void MessageReceived(const symmetric_key_crypto::cipher_array::ConstPtr& _message)
  {
    ROS_INFO("Alice -> Bob: %s", VectorToString(_message -> cipherArray).c_str());
    std::vector<int32_t> _vector = _message -> cipherArray;
    _vector.push_back(rand() % 10);
    symmetric_key_crypto::cipher_array _to_be_sent;
    _to_be_sent.cipherArray = _vector;
    m_Publisher.publish(_to_be_sent);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc,argv,"array_node_bob");
  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<symmetric_key_crypto::cipher_array>("message_for_alice",1000);
  PubSubHandler pubsubHandler = publisher;

  ros::Subscriber subscriber = nodeHandle.subscribe("message_from_alice",1000,&PubSubHandler::MessageReceived,&pubsubHandler);
  ros::spin();
  return EXIT_SUCCESS;
}

