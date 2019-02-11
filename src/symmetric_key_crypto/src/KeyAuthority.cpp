#include "ros/ros.h"
#include "symmetric_key_crypto/key_generator.h"
#include "symmetric_key_crypto/Matrix.hpp"
#include <unordered_map>
#include <string>

algebra::Matrix<int32_t> KeyGen()
{
  std::time_t _time;
  std::srand((unsigned) std::time(&_time));
  algebra::Matrix<int32_t> _identity = algebra::Identity(3);
  _identity.ElementaryRowOperation_AdditionOfAnotherMultipliedByScalar_ByIndex(0,std::rand() % 50,2);
  return _identity;
}

std::unordered_map<std::string,std::vector<int32_t>> crypto_registry;
const std::string kNodeAlice = "kNodeAlice";
const std::string kNodeBob = "kNodeBob";
const std::string nodeNameAlice = "/node_alice";
const std::string nodeNameBob = "/node_bob";

bool ServiceCalled(symmetric_key_crypto::key_generator::Request& _request,symmetric_key_crypto::key_generator::Response& _response)
{
  ROS_INFO("Key Authority is consulted by: [%s]",_request.node_id.c_str());
  if(_request.node_id == nodeNameAlice)
  {
    _response.key = crypto_registry[kNodeAlice];
    return true;
  }else if(_request.node_id == nodeNameBob)
  {
    _response.key = crypto_registry[kNodeBob];
    return true;
  }else
  {
    return false;
  }
}

int main(int argc, char **argv)
{
  if(crypto_registry.find(kNodeAlice) == crypto_registry.end() || crypto_registry.find(kNodeBob) == crypto_registry.end())
  {
    const algebra::Matrix<int32_t> _encryption_key = KeyGen();
    const std::vector<int32_t> _encryption_vector = algebra::MatrixToVector(_encryption_key,algebra::ExpansionType::E_AlongColumn);
    const algebra::Matrix<int32_t> _decryption_key = algebra::Invert(_encryption_key);
    const std::vector<int32_t> _decryption_vector = algebra::MatrixToVector(_decryption_key,algebra::ExpansionType::E_AlongColumn);
    crypto_registry[kNodeAlice] = _encryption_vector;
    crypto_registry[kNodeBob] = _decryption_vector;
  }
  ros::init(argc,argv,"key_generator");
  ros::NodeHandle nodeHandle;
  ros::ServiceServer server = nodeHandle.advertiseService("generate_key",ServiceCalled);
  ROS_INFO("Key Authority stand by...");
  ros::spin();
  return EXIT_SUCCESS;
}
