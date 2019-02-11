#ifndef MATRIXCOMPONENTS_H
#define MATRIXCOMPONENTS_H

#include "ElementSequence.hpp"

//Row Definition
namespace algebra
{
  template<typename RealNumericValueType>
  struct Row: public ElementSequence<RealNumericValueType>
  { 
    static_assert(std::is_same<RealNumericValueType,int>::value || 
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value || 
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");

    #pragma mark Public constructors
    public:
    Row() = delete;
    Row(const std::vector<RealNumericValueType>& _row_ElementSequence)
    :ElementSequence<RealNumericValueType>(_row_ElementSequence){}
    Row(const std::initializer_list<RealNumericValueType>& _m_RowList)
    :ElementSequence<RealNumericValueType>(_m_RowList){}
    Row(const algebra::Row<RealNumericValueType>& _row)
    :ElementSequence<RealNumericValueType>(_row){}
    ~Row(){}
  };

  #pragma mark Operator overloaded functions for Row
  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const algebra::Row<RealNumericValueType>& _row)
  {
    for(size_t _index = 0; _index < _row.Size(); _index += 1)
    {
      if (_index < (_row.Size() - 1)) _stream << _row[_index] << " ";
      else _stream << _row[_index];
    }
    return _stream;
  }

  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const std::vector<algebra::Row<RealNumericValueType>>& _rows)
  {
    std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row) {
      _stream << "{" << _row << "}" << " ";
    });
    return _stream;
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator +(const algebra::Row<RealNumericValueType>& _lhs, const algebra::Row<RealNumericValueType>& _rhs)
  {
    if(_lhs.Size() != _rhs.Size())
      throw std::invalid_argument("Two rows of unequal size can not be added");
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),_rhs.begin(),std::back_inserter(_buffer),[&](const RealNumericValueType& _l_Element, const RealNumericValueType& _r_Element) {
      return _l_Element + _r_Element;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator -(const algebra::Row<RealNumericValueType>& _lhs, const algebra::Row<RealNumericValueType>& _rhs)
  {
    if(_lhs.Size() != _rhs.Size())
      throw std::invalid_argument("Two rows of unequal size can not be added");
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),_rhs.begin(),std::back_inserter(_buffer),[&](const RealNumericValueType& _l_Element, const RealNumericValueType& _r_Element) {
      return _l_Element - _r_Element;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator +(const algebra::Row<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element + _scalar;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator +(const RealNumericValueType& _scalar, const algebra::Row<RealNumericValueType>& _rhs)
  {
    return _rhs + _scalar;
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator -(const algebra::Row<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element - _scalar;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator -(const RealNumericValueType& _scalar, const algebra::Row<RealNumericValueType>& _rhs)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_rhs.Size());
    std::transform(_rhs.begin(),_rhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _scalar - _element;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator *(const algebra::Row<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element * _scalar;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator *(const RealNumericValueType& _scalar, const algebra::Row<RealNumericValueType>& _rhs)
  {
    return _rhs * _scalar;
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator /(const algebra::Row<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element / _scalar;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Row<RealNumericValueType> operator %(const algebra::Row<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element % _scalar;
    });
    return algebra::Row<RealNumericValueType>(_buffer);
  }
} // algebra




//Column Definition
namespace algebra
{
  template<typename RealNumericValueType>
  struct Column: public ElementSequence<RealNumericValueType>
  {
    static_assert(std::is_same<RealNumericValueType,int>::value || 
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value || 
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");

    #pragma mark Public constructors
    public:
    Column() = delete;
    Column(const std::vector<RealNumericValueType>& _column_ElementSequence)
    :ElementSequence<RealNumericValueType>(_column_ElementSequence){}
    Column(const std::initializer_list<RealNumericValueType>& _mColumnList)
    :ElementSequence<RealNumericValueType>(_mColumnList){}
    Column(const Column<RealNumericValueType>& _column)
    :ElementSequence<RealNumericValueType>(_column){}
    ~Column(){}
  };

  #pragma mark Operator overloaded functions for Column
  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const algebra::Column<RealNumericValueType>& _column)
  {
    for(size_t _index = 0; _index < _column.Size(); _index += 1)
    {
      if (_index < (_column.Size() - 1)) _stream << _column[_index] << " ";
      else _stream << _column[_index];
    }
    return _stream;
  }

  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const std::vector<algebra::Column<RealNumericValueType>>& _columns)
  {
    std::for_each(_columns.begin(),_columns.end(),[&](const algebra::Column<RealNumericValueType>& _column) {
      _stream << "{" << _column << "}" << " ";
    });
    return _stream;
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator +(const algebra::Column<RealNumericValueType>& _lhs, const algebra::Column<RealNumericValueType>& _rhs)
  {
    if(_lhs.Size() != _rhs.Size())
      throw std::invalid_argument("Two columns of unequal size can not be added");
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),_rhs.begin(),std::back_inserter(_buffer),[&](const RealNumericValueType& _l_Element, const RealNumericValueType& _r_Element) {
      return _l_Element + _r_Element;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator -(const algebra::Column<RealNumericValueType>& _lhs, const algebra::Column<RealNumericValueType>& _rhs)
  {
    if(_lhs.Size() != _rhs.Size())
      throw std::invalid_argument("Two columns of unequal size can not be added");
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),_rhs.begin(),std::back_inserter(_buffer),[&](const RealNumericValueType& _l_Element, const RealNumericValueType& _r_Element) {
      return _l_Element - _r_Element;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator +(const algebra::Column<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element + _scalar;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator +(const RealNumericValueType& _scalar, const algebra::Column<RealNumericValueType>& _rhs)
  {
    return _rhs + _scalar;
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator -(const algebra::Column<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element - _scalar;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator -(const RealNumericValueType& _scalar, const algebra::Column<RealNumericValueType>& _rhs)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_rhs.Size());
    std::transform(_rhs.begin(),_rhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _scalar - _element;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator *(const algebra::Column<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element * _scalar;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator *(const RealNumericValueType& _scalar, const algebra::Column<RealNumericValueType>& _rhs)
  {
    return _rhs * _scalar;
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator /(const algebra::Column<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element / _scalar;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  algebra::Column<RealNumericValueType> operator %(const algebra::Column<RealNumericValueType>& _lhs, const RealNumericValueType& _scalar)
  {
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs.Size());
    std::transform(_lhs.begin(),_lhs.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element) {
      return _element % _scalar;
    });
    return algebra::Column<RealNumericValueType>(_buffer);
  }

  template<typename RealNumericValueType>
  RealNumericValueType operator *(const algebra::Row<RealNumericValueType>& _lhs_row, const algebra::Column<RealNumericValueType>& _rhs_column)
  {
    if(_lhs_row.Size() == 0 || _rhs_column.Size() == 0)
      throw std::logic_error("Can not operate on row or columns having no elements.");
    else if(_lhs_row.Size() != _rhs_column.Size())
      throw std::logic_error("Can not operate on row or column of different sizes.");
    std::vector<RealNumericValueType> _buffer;
    _buffer.reserve(_lhs_row.Size());
    std::transform(_lhs_row.begin(),_lhs_row.end(),_rhs_column.begin(),std::back_inserter(_buffer),[&](const RealNumericValueType& _lhs_element, const RealNumericValueType& _rhs_element){
      return _lhs_element * _rhs_element;
    });
    const RealNumericValueType _matrix_element = std::accumulate(_buffer.begin(),_buffer.end(),0.0);
    return _matrix_element;
  } 
} // algebra



#endif //MATRIXCOMPONENTS_H