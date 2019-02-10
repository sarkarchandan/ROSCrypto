#ifndef TEMP_KEY_ARCHIVE
#define TEMP_KEY_ARCHIVE

#include "symmetric_key_crypto/Matrix.hpp"

algebra::Matrix<int32_t> ProvideKey()
{
  const algebra::Matrix<int32_t> key = {
    {-3,-3,-4},
    {0,1,1},
    {4,3,4}
  };
  return key;
}

#endif //TEMP_KEY_ARCHIVE
