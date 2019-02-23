#include "ros/ros.h"
#include "node_alice/cipher.h"
#include "node_alice/Crypto.hpp"
#include "node_alice/MessageArchive.hpp"
#include "node_alice/key_generator.h"

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
    node_alice::cipher message_for_bob;
    message_for_bob.cipher = PrepareMessage();
    m_Publisher.publish(message_for_bob);
  }
  
  void MessageReceived(const node_alice::cipher::ConstPtr& _message)
  {
    ProcessMessage(_message);
    node_alice::cipher message_for_bob;
    message_for_bob.cipher = PrepareMessage();
    m_Publisher.publish(message_for_bob);
  }

  private:
  void ProcessMessage(const node_alice::cipher::ConstPtr& _message)
  {
    ROS_INFO("Bob -> Alice [Encrypted]: [%s]",VectorToString(_message -> cipher).c_str());
    m_Rate.sleep();
    const std::vector<int32_t> cipher_vector = _message -> cipher;
    const algebra::Matrix<int32_t> decryption_key = m_Key;
    const std::string decrypted_message = Decrypt(cipher_vector,decryption_key);
    ROS_INFO("Bob -> Alice [Decrypted]: %s",decrypted_message.c_str());
  }

  std::vector<int32_t> PrepareMessage()
  {
    const std::string _random_message = RandomMessage('A');
    const std::vector<int32_t> _cipher_vector = Encrypt(_random_message,m_Key);
    return _cipher_vector;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc,argv,"node_alice");
  ros::NodeHandle nodeHandle;
  
  #pragma mark Attemp to invoke key_generator service
  ros::ServiceClient client = nodeHandle.serviceClient<node_alice::key_generator>("generate_key");
  node_alice::key_generator generator;
  generator.request.node_id = ros::this_node::getName();
  
  if(client.call(generator))
  {
    ros::Rate rate(1);
    ros::Publisher publisher = nodeHandle.advertise<node_alice::cipher>("message_from_alice",10,true);
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

