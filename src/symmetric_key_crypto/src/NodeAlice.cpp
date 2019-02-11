#include "ros/ros.h"
#include "symmetric_key_crypto/cipher_array.h"
#include "symmetric_key_crypto/Crypto.hpp"
#include "symmetric_key_crypto/MessageArchive.hpp"
#include "symmetric_key_crypto/TempKeyArchive.hpp"
#include "symmetric_key_crypto/key_generator.h"

class CryptoPubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  ros::Rate m_Rate;
  algebra::Matrix<int32_t> m_Key;
  public:
  CryptoPubSubHandler(const ros::Publisher& _publisher,const ros::Rate _rate,const algebra::Matrix<int32_t> _m_Key)
  :m_Publisher(_publisher),m_Rate(_rate),m_Key(_m_Key)
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
    ROS_DEBUG("Bob -> Alice [Encrypted]: [%s]",VectorToString(_message -> cipherArray).c_str());
    m_Rate.sleep();
    const std::vector<int32_t> cipher_vector = _message -> cipherArray;
    const algebra::Matrix<int32_t> decryption_key = m_Key;
    const std::string decrypted_message = Decrypt(cipher_vector,decryption_key);
    ROS_INFO("Bob -> Alice [Decrypted]: %s",decrypted_message.c_str());
  }

  std::vector<int32_t> PrepareMessage()
  {
    const std::string _random_message = RandomMessage('A');
    const std::vector<int32_t> _cipher_vector = Encrypt(_random_message,m_Key);
    ROS_DEBUG("Bob -> Alice [Encrypted]: [%s]",VectorToString(_cipher_vector).c_str());
    return _cipher_vector;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"node_alice");
  ros::NodeHandle nodeHandle;
  
  #pragma mark Attemp to invoke key_generator service
  ros::ServiceClient client = nodeHandle.serviceClient<symmetric_key_crypto::key_generator>("generate_key");
  symmetric_key_crypto::key_generator generator;
  generator.request.node_id = ros::this_node::getName();
  
  if(client.call(generator))
  {
    ros::Rate rate(1);
    ros::Publisher publisher = nodeHandle.advertise<symmetric_key_crypto::cipher_array>("message_from_alice",10,true);
    const std::vector<int32_t> key_vector = generator.response.key;
    const algebra::Matrix<int32_t> key_matrix = algebra::VectorToMatrix(key_vector,algebra::ContractionType::C_AlongColumn,std::make_pair<size_t,size_t>(3,3));
    CryptoPubSubHandler crypto = {publisher,rate,key_matrix};
    ros::Subscriber subscriber = nodeHandle.subscribe("message_for_alice",1000,&CryptoPubSubHandler::MessageReceived,&crypto);
    ros::spin();
  }else
  {
    exit(EXIT_FAILURE);
  }
  return EXIT_SUCCESS;
}

