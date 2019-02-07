#ifndef VECTOR_HANDLER_H
#define VECTOR_HANDLER_H

#include <vector>
#include <string>


std::string VectorToString(const std::vector<int32_t>& _vector)
{
  std::string _string = "{";
  for(size_t _index = 0; _index < _vector.size(); _index += 1)
  {
    (_index < (_vector.size() - 1)) ? _string += std::to_string(_vector[_index]) + "," : _string += std::to_string(_vector[_index]);
  }
  _string += "}";
  return _string;
}

#endif //VECTOR_HANDLER_H
