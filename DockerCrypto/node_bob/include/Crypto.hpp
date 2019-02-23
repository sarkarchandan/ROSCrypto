#ifndef CRYPTO_H
#define CRYPTO_H

#include "Matrix.hpp"
#include "CharacterAnalyzer.hpp"
#include <string>

template<typename T>
std::string VectorToString(const std::vector<T>& _vector)
{
  std::string _string = "{";
  for(size_t _index = 0; _index < _vector.size(); _index += 1)
  {
    (_index < (_vector.size() - 1)) ? _string += std::to_string(_vector[_index]) + "," : _string += std::to_string(_vector[_index]);
  }
  _string += "}";
  return _string;
}

std::vector<int32_t> Encrypt(const std::string& _message, const algebra::Matrix<int32_t>& _encryption_key)
{
  const size_t _number_of_placeholders = NumberOfPlaceHolderNeeded(_message.length());
  std::vector<int32_t> _cipher_vector;
  _cipher_vector.reserve(_message.length() + _number_of_placeholders);
  for(size_t _index = 0; *(_message.c_str() + _index) != '\0'; _index += 1)
    _cipher_vector.emplace_back(DeriveCodeForCharacter(*(_message.c_str() + _index)));
  if(_number_of_placeholders > 0)
  {
    for(size_t _index = 0; _index < _number_of_placeholders; _index += 1)
      _cipher_vector.emplace_back(26);
  }
  const algebra::Matrix<int32_t> _derived_matrix = algebra::VectorToMatrix(_cipher_vector,algebra::ContractionType::C_AlongColumn,std::make_pair<size_t,size_t>(3,(_message.length() + _number_of_placeholders)/3));
  const algebra::Matrix<int32_t> _encrypted_matrix =  _encryption_key * _derived_matrix;
  return algebra::MatrixToVector(_encrypted_matrix,algebra::ExpansionType::E_AlongColumn);
}

std::string Decrypt(const std::vector<int32_t>& _cipher_vector, const algebra::Matrix<int32_t>& _decryption_key)
{
  const char uppercaseBuffer[] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z',' ',',','.'};
const char lowercaseBuffer[] = {'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z',' ',',','.'};
  const algebra::Matrix<int32_t> _cipher_matrix = algebra::VectorToMatrix(_cipher_vector,algebra::ContractionType::C_AlongColumn,std::make_pair<size_t,size_t>(3,(_cipher_vector.size() / 3)));
  const algebra::Matrix<int32_t> _decrypted_matrix = _decryption_key * _cipher_matrix;
  const std::vector<int32_t> _decrypted_vector = algebra::MatrixToVector(_decrypted_matrix,algebra::ExpansionType::E_AlongColumn);
  std::string _recovered_message = "";
  for(size_t _index = 0; _index < _decrypted_vector.size(); _index += 1)
  {
    if(_index == 0)
      _recovered_message += *(uppercaseBuffer + _decrypted_vector[_index]);
    else 
      _recovered_message += *(lowercaseBuffer + _decrypted_vector[_index]);
  }
  return _recovered_message;
}

#endif //CRYPTO_H
