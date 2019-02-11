#ifndef MULTISEQUENCE_H
#define MULTISEQUENCE_H

#include <type_traits>
#include <vector>
#include <exception>
#include <algorithm>
#include <numeric>
#include <typeinfo>
#include <initializer_list>
#include <memory>
#include <functional>
#include "MultiSequenceComponents.hpp"
#include "ElementProtocol.hpp"

namespace algebra
{
  template<typename RealNumericValueType>
  struct MultiSequence
  {
    static_assert(std::is_same<RealNumericValueType,int>::value ||
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value ||
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");

    #pragma mark Protected member properties
    protected:
    std::unique_ptr<std::vector<std::vector<RealNumericValueType>>> m_Container;

    #pragma mark Protected helper functions
    protected:
    virtual bool IsValid(const std::initializer_list<std::vector<RealNumericValueType>>& _init_list) const
    {
      std::vector<size_t> _buffer;
      _buffer.reserve(_init_list.size());
      std::transform(_init_list.begin(),_init_list.end(),std::back_inserter(_buffer),[&](const std::vector<RealNumericValueType>& _collection) {
        return _collection.size();
      });
      return std::adjacent_find(_buffer.begin(),_buffer.end(),std::not_equal_to<size_t>()) == _buffer.end();
    }

    virtual bool IsValid(const std::vector<std::vector<RealNumericValueType>>& _vectors) const
    {
      std::vector<size_t> _buffer;
      _buffer.reserve(_vectors.size());
      std::transform(_vectors.begin(),_vectors.end(),std::back_inserter(_buffer),[&](const std::vector<RealNumericValueType>& _collection) {
        return _collection.size();
        });
      return std::adjacent_find(_buffer.begin(),_buffer.end(),std::not_equal_to<size_t>()) == _buffer.end();
    }

    virtual bool IsValid(const std::vector<algebra::Row<RealNumericValueType>>& _rows) const
    {
      std::vector<size_t> _buffer;
      _buffer.reserve(_rows.size());
      std::transform(_rows.begin(),_rows.end(),std::back_inserter(_buffer),[&](const algebra::Row<RealNumericValueType>& _row) {
        return _row.Size();
      });
      return std::adjacent_find(_buffer.begin(),_buffer.end(),std::not_equal_to<size_t>()) == _buffer.end();
    }

    virtual bool IsValid(const std::vector<algebra::Column<RealNumericValueType>>& _columns) const
    {
      std::vector<size_t> _buffer;
      _buffer.reserve(_columns.size());
      std::transform(_columns.begin(),_columns.end(),std::back_inserter(_buffer),[&](const algebra::Column<RealNumericValueType>& _column){
        return _column.Size();
      });
      return std::adjacent_find(_buffer.begin(),_buffer.end(),std::not_equal_to<size_t>()) == _buffer.end();
    }

    void SetRowByIndex(const algebra::Row<RealNumericValueType>& _row, const size_t& _row_index)
    {
      for(size_t _column_index = 0; _column_index < Order().second; _column_index += 1)
          operator()(_row_index,_column_index) = _row[_column_index];
    }

    void SetColumnByIndex(const algebra::Column<RealNumericValueType>& _column, const size_t& _column_index)
    {
      for(size_t _row_index = 0; _row_index < Order().first; _row_index += 1)
          operator()(_row_index,_column_index) = _column[_row_index];
    }

    #pragma mark protected constructors to be accessed only by child classes
    protected:
    MultiSequence(){}
    MultiSequence(const std::initializer_list<std::vector<RealNumericValueType>>& _init_list)
    {
      if (!IsValid(_init_list))
        throw std::invalid_argument("Unequal number of elements in list");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_init_list.size());
      std::for_each(_init_list.begin(),_init_list.end(),[&](const std::vector<RealNumericValueType>& _element_vector) {
          this -> m_Container -> emplace_back(_element_vector);
        });
    }
    MultiSequence(const std::vector<std::vector<RealNumericValueType>>& _vectors)
    {
      if (!IsValid(_vectors))
        throw std::invalid_argument("Unequal number of elements in vectors");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_vectors.size());
      std::for_each(_vectors.begin(),_vectors.end(),[&](const std::vector<RealNumericValueType>& _element_vector) {
        this -> m_Container -> emplace_back(_element_vector);
      });
    }
    MultiSequence(const std::vector<algebra::Row<RealNumericValueType>>& _rows)
    {
      if(!IsValid(_rows))
        throw std::invalid_argument("Unequal number of elements in rows");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_rows.size());
      std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row){
        std::vector<RealNumericValueType> _buffer;
        _buffer.reserve(_row.Size());
        std::transform(_row.begin(),_row.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element){
          return _element;
        });
        this -> m_Container -> emplace_back(_buffer);
      });
    }
    MultiSequence(const std::vector<algebra::Column<RealNumericValueType>>& _columns)
    {
      if(!IsValid(_columns))
        throw std::invalid_argument("Unequal number of elements in columns");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_columns[0].Size());
      for(size_t _index = 0; _index < _columns[0].Size(); _index += 1)
      {
        std::vector<RealNumericValueType> _row_buffer;
        _row_buffer.reserve(_columns.size());
        std::for_each(_columns.begin(),_columns.end(),[&](const algebra::Column<RealNumericValueType>& _column){
          _row_buffer.emplace_back(_column[_index]);
        });
        this -> m_Container -> emplace_back(_row_buffer);
      }
    }
    MultiSequence(const MultiSequence& _multiSequence)
    {
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_multiSequence.Order().first);
      std::vector<algebra::Row<RealNumericValueType>> _rows = _multiSequence.Rows();
      std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row) {
        std::vector<RealNumericValueType> _buffer;
        _buffer.reserve(_row.Size());
        std::for_each(_row.begin(),_row.end(),[&](const RealNumericValueType& _element) {
          _buffer.emplace_back(_element);
        });
        this -> m_Container -> emplace_back(_buffer);
      });
    }
    ~MultiSequence(){}

    #pragma mark Public member functions and accessors
    public:
    inline std::pair<size_t,size_t> Order() const
    {
      const std::pair<size_t,size_t> _pair = std::make_pair<size_t,size_t>(this -> m_Container -> size(),this -> m_Container -> operator[](0).size());
      return _pair;
    }

    inline const RealNumericValueType& operator ()(const size_t& _row,const size_t& _column) const
    {
      if(_row >= 0 && _row < Order().first && _column >= 0 && _column < Order().second)
        return this -> m_Container -> operator[](_row)[_column];
      else
        throw std::out_of_range("Out of range subscripting attempted");
    }

    inline RealNumericValueType& operator ()(const size_t& _row, const size_t& _column)
    {
      if(_row >= 0 && _row < Order().first && _column >= 0 && _column < Order().second)
        return this -> m_Container -> operator[](_row)[_column];
      else
        throw std::out_of_range("Out of range subscripting attempted");
    }

    const std::vector<algebra::Row<RealNumericValueType>> Rows() const
    {
      std::vector<algebra::Row<RealNumericValueType>> _rows;
      _rows.reserve(Order().first);
      std::for_each(this -> m_Container -> begin(),this -> m_Container -> end(),[&](const std::vector<RealNumericValueType>& _row){
        _rows.emplace_back(_row);
      });
      return _rows;
    }

    const std::vector<algebra::Column<RealNumericValueType>> Columns() const
    {
      std::vector<algebra::Column<RealNumericValueType>> _columns;
      _columns.reserve(Order().second);
      std::vector<RealNumericValueType> _columnBuffer(Order().first);

      for(size_t column_index = 0; column_index < Order().second; column_index += 1)
      {
        _columnBuffer.clear();
        for(size_t row_index = 0; row_index < Order().first; row_index += 1)
        {
          _columnBuffer.emplace_back(this -> operator()(row_index,column_index));
        }
        _columns.emplace_back(_columnBuffer);
      }
      return _columns;
    }

    MultiSequence& operator =(const algebra::MultiSequence<RealNumericValueType>& _sequence)
    {
      if(this == &_sequence)
        return *this;
      if(this -> Order() != _sequence.Order())
        throw std::length_error("Copy assignment attempted for matrices of different order.");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_sequence.Order().first);
      std::for_each(_sequence.m_Container -> begin(),_sequence.m_Container -> end(),[&](const std::vector<RealNumericValueType>& _row){
        this -> m_Container -> emplace_back(_row);
      });
      return *this;
    }

    void ElementaryRowOperation_Interchange_ByIndex(const size_t& _fist_row_index, const size_t& _second_row_index)
    {
      if(!(_fist_row_index >= 0 && _fist_row_index < this -> Order().first) || !(_second_row_index >= 0 && _second_row_index < this -> Order().first))
        throw std::out_of_range("One or both of the row indices are out of range");
      std::vector<algebra::Row<RealNumericValueType>> _rows = this -> Rows();
      this -> SetRowByIndex(_rows[_fist_row_index],_second_row_index);
      this -> SetRowByIndex(_rows[_second_row_index],_fist_row_index);
    }

    void ElementaryRowOperation_MultiplicationByNonZeroScalar_ByIndex(const RealNumericValueType& _scalar, const size_t& _row_index)
    {
      if(!(_row_index >= 0 && _row_index < this -> Order().first))
        throw std::out_of_range("Row index is out of range");
      const algebra::Row<RealNumericValueType> _row = this -> Rows()[_row_index];
      this -> SetRowByIndex(_scalar * _row, _row_index);
    }

    void ElementaryRowOperation_AdditionOfAnotherMultipliedByScalar_ByIndex(const size_t& _target_row_index,const RealNumericValueType& _scalar, const size_t& _source_row_index)
    {
      if(!(_target_row_index >= 0  && _target_row_index < this -> Order().first) || !(_source_row_index >= 0 && _source_row_index < this -> Order().first))
        throw std::out_of_range("One or both of source and target row indices are out of range");
      const algebra::Row<RealNumericValueType> _source_row = this -> Rows()[_source_row_index];
      const algebra::Row<RealNumericValueType> _target_row = this -> Rows()[_target_row_index];
      this -> SetRowByIndex((_target_row + (_scalar * _source_row)),_target_row_index);
    }

    void ElementaryColumnOperation_Interchange_ByIndex(const size_t& _first_column_index, const size_t& _second_column_index)
    {
      if(!(_first_column_index >= 0 && _first_column_index < this -> Order().second) || !(_second_column_index >= 0 && _second_column_index < this -> Order().second))
        throw std::out_of_range("One or both of the column indices are out of range");
      std::vector<algebra::Column<RealNumericValueType>> _columns = this -> Columns();
      this -> SetColumnByIndex(_columns[_first_column_index],_second_column_index);
      this -> SetColumnByIndex(_columns[_second_column_index],_first_column_index);
    }

    void ElementaryColumnOperation_MultiplicationByNonZeroScalar_ByIndex(const RealNumericValueType& _scalar, const size_t& _column_index)
    {
      if(!(_column_index >= 0 && _column_index < this -> Order().second))
        throw std::out_of_range("Column index is out of range");
      const algebra::Column<RealNumericValueType> _column = this -> Columns()[_column_index];
      this -> SetColumnByIndex(_scalar * _column,_column_index);
    }

    void ElementaryColumnOperation_AdditionOfAnotherMultipliedByScalar_ByIndex(const size_t& _target_column_index, const RealNumericValueType& _scalar, const size_t& _source_column_index)
    {
      if(!(_target_column_index >= 0 && _target_column_index < this -> Order().second) || !(_source_column_index >= 0 && _source_column_index < this -> Order().second))
        throw std::out_of_range("One or both of source and target column indices are out of range");
      const algebra::Column<RealNumericValueType> _source_column = this -> Columns()[_source_column_index];
      const algebra::Column<RealNumericValueType> _target_column = this -> Columns()[_target_column_index];
      this -> SetColumnByIndex((_target_column + (_scalar * _source_column)),_target_column_index);
    }
  };

  template<typename RealNumericValueType>
  bool operator ==(const algebra::MultiSequence<RealNumericValueType>& _lhs, const algebra::MultiSequence<RealNumericValueType>& _rhs)
  {
    if(_lhs.Order() != _rhs.Order())
      return false;

    const std::vector<algebra::Row<RealNumericValueType>> _lhs_Rows = _lhs.Rows();
    const std::vector<algebra::Row<RealNumericValueType>> _rhs_Rows = _rhs.Rows();

    return std::equal(_lhs_Rows.begin(),_lhs_Rows.end(),_rhs_Rows.begin(),_rhs_Rows.end(),[&](const algebra::Row<RealNumericValueType>& _lhs_Row, const algebra::Row<RealNumericValueType>& _rhs_Row) {
      return _lhs_Row == _rhs_Row;
    });
  }

  template<typename RealNumericValueType>
  bool operator !=(const algebra::MultiSequence<RealNumericValueType>& _lhs, const algebra::MultiSequence<RealNumericValueType>& _rhs)
  {
    return !(_lhs == _rhs);
  }

  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const std::vector<RealNumericValueType>& _vector)
  {
    _stream << "{ ";
    std::for_each(_vector.begin(),_vector.end(),[&](const RealNumericValueType& _element){
      _stream << _element << " ";
    });
    _stream << "}";
    return _stream;
  }

  template<typename RealNumericValueType>
  std::ostream& operator <<(std::ostream& _stream, const algebra::MultiSequence<RealNumericValueType>& _sequence)
  {
    _stream << "\n";
    std::vector<algebra::Row<RealNumericValueType>> _rows = _sequence.Rows();
    std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row) {
      _stream << _row << "\n";
    });
    _stream << "\n";
    return _stream;
  }

} // algebra


#endif //MULTISEQUENCE_H
