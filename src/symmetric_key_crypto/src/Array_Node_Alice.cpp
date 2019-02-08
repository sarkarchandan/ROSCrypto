#include "ros/ros.h"
#include "symmetric_key_crypto/cipher_array.h"
#include <cstdlib>
#include "symmetric_key_crypto/VectorHandler.h"
#include "symmetric_key_crypto/Matrix.hpp"

class PubSubHandler
{
  private:
  ros::Publisher m_Publisher;
  ros::Rate m_Rate;
  public:
  PubSubHandler(const ros::Publisher& _publisher,const ros::Rate _rate)
  :m_Publisher(_publisher),m_Rate(_rate)
  {
    symmetric_key_crypto::cipher_array _message;
    const algebra::Matrix<int32_t> matrix1 = {
      {1,2,3},
      {4,5,6},
      {7,8,9}
    };
    const std::vector<int32_t> _vector = algebra::MatrixToVector(matrix1,algebra::ExpansionType::E_AlongRow);
    _message.cipherArray = _vector;
    m_Publisher.publish(_message);
  }
  
  void MessageReceived(const symmetric_key_crypto::cipher_array::ConstPtr& _message)
  {
    ROS_INFO("Bob -> Alice: %s",VectorToString(_message -> cipherArray).c_str());
    m_Rate.sleep();
    std::vector<int32_t> _vector = _message -> cipherArray;
    const algebra::Matrix<int32_t> _matrix = algebra::VectorToMatrix(_vector,algebra::ContractionType::C_AlongRow,std::make_pair<size_t,size_t>(3,3));
    const algebra::Matrix<int32_t> _transposed = _matrix.Transpose();
    const std::vector<int32_t> _transformed_vector = algebra::MatrixToVector(_transposed,algebra::ExpansionType::E_AlongRow);
    symmetric_key_crypto::cipher_array _to_be_sent;
    _to_be_sent.cipherArray = _transformed_vector;
    m_Publisher.publish(_to_be_sent);
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

