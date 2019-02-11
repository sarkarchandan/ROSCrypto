# ROSCrypto

#### This is a C++ prototypical implementation for a simple example of cryptography built using some of the basic building blocks of [ROS](http://www.ros.org "Robot Operating System"). The encryption and decryption methods are adopted from a simplified version of [Hill cipher](https://en.wikipedia.org/wiki/Hill_cipher "A polygraphic substitution cipher"). 

#### In the sender node, a plain text message is transformed into an std::vector\<int32_t> with the help of ASCII value of each character. Some of the punctuation symbols are ignored for the sake of simplicity. A matrix of order 3xn is constructed from the vector such that each consecutive set of three numbers became a column vector in the matrix. A chosen encryption key matrix of order 3x3 is then multiplied with the generated 3xn matrix in order to derive the cipher matrix. The sender node publishes the cipher in a flattened serialized form using a custom ROS message. 

#### The receiver node receives the cipher upon subscribing to the specific ROS message and reconstructs a 3xn matrix from the flattened cipher. A compatible decryption key matrix of order 3x3 is then multiplied with the cipher matrix in order to recover the matrix representing the plaintext message in numeric form, which is finally transformed into the original plaintext message.


#### The library [Francois](https://github.com/sarkarchandan/Francois "This is a library written in C++, using STL containers and algorithms in order to provide implementations for some of the core structures of the linear algebra") is used to perform all the associated matrix operations. This implementation features three _ROS_ [_Nodes_](http://wiki.ros.org/Nodes "The fundamental building block of computation in ROS").

## node: key_authority

##### Node _key\_authority_ implements a ROS [_Service_](http://wiki.ros.org/Services "ROS building block to support the remote procedure call paradigm") which is responsible for generating a pair of compatible unique keys for a given peer to peer communication session between _node\_alice_ and _node\_bob_. It then distributes the keys between the two nodes on demand. The key generation is a simple procedure of constructing an elementary matrix of order 3x3 for each communication session, using the elementary operations and utility methods implemented in _Francois_. KeyAuthority leverages the invertibility property of the elementary matrices. Thus elementary matrix and its inverse constitute the encryption key and decryption key respectively or vice versa. These keys are in turn distributed between the _node\_alice_ and _node\_bob_ over service call.

```cpp
//Creates an elementary matrix of order 3x3 to be used to generate encryption and decryption keys 
algebra::Matrix<int32_t> GenerateKey()
{
  std::time_t _time;
  std::srand((unsigned) std::time(&_time));
  algebra::Matrix<int32_t> _identity = algebra::Identity(3);
  //Elementary row operation: Row[0] -> Row[0] + (std::rand() % 50) * Row[2]
  _identity.ElementaryRowOperation_AdditionOfAnotherMultipliedByScalar_ByIndex(0,std::rand() % 50,2);
  return _identity;
}

```

## nodes: node\_alice & node\_bob

##### The nodes _node\_alice_ and _node\_bob_ are typical ROS nodes which, are engaged in the peer to peer communication, by choosing sample plaintexts from a pool of predefined messages. Upon initializing the environment, both nodes call the _key\_authority_ service in order to collect their individual keys. Because of the fact that their individually collected keys are basically inverse of each other, any of the nodes can use its own key to encrypt a plaintext message and the other node is capable to decrypt the same and recover the message. 

##### Encryption function looks like this

```cpp
/*
* Encrypts an std::string plaintext into an std::vector<int32_t> cipher using
* algebra::Matrix<int32_t> key.
* size_t NumberOfPlaceHolderNeeded(const size_t&) computes the number of
* placeholder digits needed to add in the vector which so that an 
* algebra::Matrix<int32_t> of order 3xn could be constructed.
* int32_t DeriveCodeForCharacter(const char& ) derives the appropriate int32_t
* digit from the passed in character using ASCII values.
*/
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

```

##### Decryption function looks like this

```cpp
/*
* Decrypts an std::vector<int32_t> cipher into std::string plaintext using 
* algebra::Matrix<int32_t> key.
* Uppercase and lowercase character buffers are used to replace the 
* characters in the plaintext string.
*/

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

```

## ROS Graph

##### The Graph representing this simple prototypical ROS system constitute two nodes namely, node\_alice and node\_bob connected with two directed edges, message\_from\_alice and message\_for\_alice respectively, as well as a disjoint service node key\_authority.

* **node_alice** publishes data of type std\_msgs/Int32[] with topic name **message\_from\_alice** and subscribes to **message\_for\_alice** with an expectation of data of type std\_msgs/Int32[].
* **node_bob** publishes data of type std\_msgs/Int32[] with topic name **message\_for\_alice** and subscribes to **message\_from\_alice** with an expectation of data of type std\_msgs/Int32[].

<img width="720" alt="crypto_rosgraph" src="https://user-images.githubusercontent.com/19269229/52596152-eb827400-2e4f-11e9-93ce-ddf95ec99221.png">


## Installation

##### 


```bash



```

## Examples

##### 

```bash
$
```

## Dependencies

## Authors


##### 

## License
##### This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
