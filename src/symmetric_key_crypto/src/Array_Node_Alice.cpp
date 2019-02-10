#include "ros/ros.h"
#include "symmetric_key_crypto/cipher_array.h"
#include "symmetric_key_crypto/Crypto.hpp"
#include "symmetric_key_crypto/MessageArchive.hpp"
#include "symmetric_key_crypto/TempKeyArchive.hpp"

class PubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  ros::Rate m_Rate;
  public:
  PubSubHandler(const ros::Publisher& _publisher,const ros::Rate _rate)
  :m_Publisher(_publisher),m_Rate(_rate)
  {
    symmetric_key_crypto::cipher_array message_for_bob;
    message_for_bob.cipherArray = PrepareMessage();
    m_Publisher.publish(message_for_bob);
  }
  
  void MessageReceived(const symmetric_key_crypto::cipher_array::ConstPtr& _message)
  {
    ProcessMessage(_message);
    symmetric_key_crypto::cipher_array message_for_bob;
    message_for_bob.cipherArray = PrepareMessage();
    m_Publisher.publish(message_for_bob);
  }

  private:
  void ProcessMessage(const symmetric_key_crypto::cipher_array::ConstPtr& _message)
  {
    //ROS_INFO("Bob -> Alice [Encrypted]: [%s]",VectorToString(_message -> cipherArray).c_str());
    m_Rate.sleep();
    const std::vector<int32_t> cipher_vector = _message -> cipherArray;
    const algebra::Matrix<int32_t> decryption_key = algebra::Invert(ProvideKey());
    const std::string decrypted_message = Decrypt(cipher_vector,decryption_key);
    ROS_INFO("Bob -> Alice [Decrypted]: %s",decrypted_message.c_str());
  }

  std::vector<int32_t> PrepareMessage()
  {
    const std::string random_message = RandomMessage('A');
    const algebra::Matrix<int32_t> encryption_key = ProvideKey();
    const std::vector<int32_t> cipher_vector = Encrypt(random_message,encryption_key);
    return cipher_vector;
  }
  
};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"array_node_alice");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(1);
  ros::Publisher publisher = nodeHandle.advertise<symmetric_key_crypto::cipher_array>("message_from_alice",10,true);
  PubSubHandler pubsubHandler = {publisher,rate};
  ros::Subscriber subscriber = nodeHandle.subscribe("message_for_alice",1000,&PubSubHandler::MessageReceived,&pubsubHandler);
  ros::spin();
  return EXIT_SUCCESS;
}

