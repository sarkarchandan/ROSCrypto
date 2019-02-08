#ifndef ELEMENTSEQUENCE_H
#define ELEMENTSEQUENCE_H

#include <type_traits>
#include <initializer_list>
#include "ElementProtocol.hpp"
#include <exception>

namespace algebra
{
  template<typename RealNumericValueType>
  struct ElementSequence
  {
    static_assert(std::is_same<RealNumericValueType,int>::value || 
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value || 
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");
    
    #pragma mark Protected member properties to be accessed only by child classes 
    protected:
    std::unique_ptr<std::vector<RealNumericValueType>> m_ElementSequence;

    #pragma mark Protected member functions to be accessed only by child classes
    protected:
    ElementSequence(){};
    ElementSequence(const std::vector<RealNumericValueType>& _m_ElementSequence)
    {
      m_ElementSequence = std::make_unique<std::vector<RealNumericValueType>>();
      m_ElementSequence -> reserve(_m_ElementSequence.size());
      std::for_each(_m_ElementSequence.begin(),_m_ElementSequence.end(),[&](const RealNumericValueType& _element){
        m_ElementSequence -> emplace_back(_element);
      });
    }
    ElementSequence(const std::initializer_list<RealNumericValueType>& _m_ElementSequence)
    {
      m_ElementSequence = std::make_unique<std::vector<RealNumericValueType>>();
      m_ElementSequence -> reserve(_m_ElementSequence.size());
      std::for_each(_m_ElementSequence.begin(),_m_ElementSequence.end(),[&](const RealNumericValueType& _element){
        m_ElementSequence -> emplace_back(_element);
      });
    }
    
    ElementSequence(const ElementSequence& _elementSequence)
    {
      m_ElementSequence = std::make_unique<std::vector<RealNumericValueType>>();
      m_ElementSequence -> reserve(_elementSequence.Size());
      std::for_each(_elementSequence.begin(),_elementSequence.end(),[&](const RealNumericValueType& _element){
        m_ElementSequence -> emplace_back(_element);
      });
    }
    ~ElementSequence(){}

    #pragma mark Public accessors and member functions
    public:
    inline const size_t Size() const { return m_ElementSequence -> size(); }
    inline const RealNumericValueType& operator[](const size_t& _index) const
    {
      return m_ElementSequence -> operator[](_index);
    }
    inline RealNumericValueType& operator[](const size_t& _index)
    {
      return m_ElementSequence -> operator[](_index);
    }
    inline typename std::vector<RealNumericValueType>::iterator begin() const
    {
      return m_ElementSequence -> begin();
    }
    inline typename std::vector<RealNumericValueType>::iterator end() const
    {
      return m_ElementSequence -> end();
    }
    inline void Erase_ByIndex(const size_t& _index) const
    {
      m_ElementSequence -> erase(m_ElementSequence -> begin() + _index);
    }
    ElementSequence& operator =(const algebra::ElementSequence<RealNumericValueType>& _elementSequence)
    {
      if(this == &_elementSequence)
        return *this;
      if((Size() != _elementSequence.Size()) || _elementSequence.Size() == 0)
        throw std::length_error("Copy assignment attempted for sequence of elements of unequal sizes.");
      m_ElementSequence = std::make_unique<std::vector<RealNumericValueType>>();
      m_ElementSequence -> reserve(_elementSequence.Size());
      std::for_each(_elementSequence.begin(),_elementSequence.end(),[&](const RealNumericValueType& _element){
        m_ElementSequence -> emplace_back(_element);
      });
      return *this;
    }
  };

  #pragma mark Operator overloaded functions for ElementSequence
  template<typename RealNumericValueType>
  bool operator ==(const algebra::ElementSequence<RealNumericValueType>& _lhs, const typename algebra::ElementSequence<RealNumericValueType>& _rhs)
  {
    if(_lhs.Size() != _rhs.Size())
      return false;
    return std::equal(_lhs.begin(),_lhs.end(),_rhs.begin(),_rhs.end(),[&](const RealNumericValueType& _lhs_element, const RealNumericValueType& _rhs_element){
      return _lhs_element == _rhs_element;
    });
  }

  template<typename RealNumericValueType>
  bool operator !=(const algebra::ElementSequence<RealNumericValueType>& _lhs, const algebra::ElementSequence<RealNumericValueType>& _rhs)
  {
    return !(_lhs == _rhs);
  }
} // algebra


#endif //ELEMENTSEQUNECE_H