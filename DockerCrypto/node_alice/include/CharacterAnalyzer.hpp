#ifndef CHARACTERANALYZER_H
#define CHARACTERANALYZER_H

#include <cstring>

inline bool IsUpperCaseLetter(const char& _character)
{
  if(_character >= 65 && _character <= 90) return true;
  else return false;
}
inline bool IsLowerCaseLetter(const char& _character)
{
  if(_character >= 97 && _character <= 122) return true;
  else return false;
}
inline bool IsSpace(const char& _character) { return _character == 32; }
inline bool IsComma(const char& _character) { return _character == 44; }
inline bool IsStop(const char& _character) { return _character == '.'; }
inline bool IsALetter(const char& _character) { return IsUpperCaseLetter(_character) || IsLowerCaseLetter(_character); }
inline bool IsPunctuation(const char& _character) { return IsSpace(_character) || IsComma(_character); }

inline int32_t DeriveCodeForCharacter(const char& _character)
{
  if(IsALetter(_character))
  {
    if(IsUpperCaseLetter(_character)) return _character - 65;
    else return _character - 97;
  }else if(IsPunctuation(_character))
  {
    if(IsSpace(_character)) return 26;
    else if(IsComma(_character)) return 27;
    else if(IsStop(_character)) return 28;
    else return -1;
  }else return -1;
}

inline size_t NumberOfPlaceHolderNeeded(const size_t& _message_length)
{
  if(_message_length % 3 == 0) return 0;
  else
  {
    size_t _temp_length = _message_length;
    do { _temp_length += 1; }while(_temp_length % 3 != 0);
    return _temp_length - _message_length;
  }
}

#endif //CHARACTERANALYZER_H