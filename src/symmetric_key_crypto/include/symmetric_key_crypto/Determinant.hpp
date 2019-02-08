#ifndef DETERMINANT_H
#define DETERMINANT_H

#include "MultiSequence.hpp"
#include <cmath>

namespace algebra
{
  template<typename RealNumericValueType>
  class Determinant: public MultiSequence<RealNumericValueType>
  {
    static_assert(std::is_same<RealNumericValueType,int>::value ||
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value ||
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");

    #pragma mark Private helper functions
    private:
    bool IsValid(const std::initializer_list<std::vector<RealNumericValueType>>& _init_list) const override
    {
      return MultiSequence<RealNumericValueType>::IsValid(_init_list) && _init_list.size() == _init_list.begin() -> size();
    }

    bool IsValid(const std::vector<std::vector<RealNumericValueType>>& _vectors)const override
    {
      return MultiSequence<RealNumericValueType>::IsValid(_vectors) && _vectors.size() == _vectors.begin() -> size();
    }

    bool IsValid(const std::vector<algebra::Row<RealNumericValueType>>& _rows)const override
    {
      return MultiSequence<RealNumericValueType>::IsValid(_rows) && _rows.size() == _rows.begin() -> Size();
    }

    bool IsValid(const std::vector<algebra::Column<RealNumericValueType>>& _columns)const override
    {
      return MultiSequence<RealNumericValueType>::IsValid(_columns) && _columns.size() == _columns.begin() -> Size();
    }

    inline bool IsEvenPower(const size_t& _lhs, const size_t& _rhs) const
    {
      if((_lhs + _rhs) == 0) return true;
      else if((_lhs + _rhs) % 2 == 0) return true;
      else return false;
    }

    #pragma mark Public constructors
    public:
    Determinant() = delete;
    Determinant(const std::initializer_list<std::vector<RealNumericValueType>>& _init_list)
    {
      if(!IsValid(_init_list))
        throw std::invalid_argument("Unequal number of elements in list");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_init_list.size());
      std::for_each(_init_list.begin(),_init_list.end(),[&](const std::vector<RealNumericValueType>& _element_vector) {
        this -> m_Container -> emplace_back(_element_vector);
      });
    }
    Determinant(const std::vector<std::vector<RealNumericValueType>>& _vectors)
    {
      if(!IsValid(_vectors))
        throw std::invalid_argument("Unequal number of elements in vectors");
      this -> m_Container = std::make_unique<std::vector<std::vector<RealNumericValueType>>>();
      this -> m_Container -> reserve(_vectors.size());
      std::for_each(_vectors.begin(),_vectors.end(),[&](const std::vector<RealNumericValueType>& _element_vector) {
        this -> m_Container -> emplace_back(_element_vector);
      });
    }
    Determinant(const std::vector<algebra::Row<RealNumericValueType>>& _rows)
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
    Determinant(const std::vector<algebra::Column<RealNumericValueType>>& _columns)
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
    Determinant(const Determinant& _determinant)
    :MultiSequence<RealNumericValueType>(_determinant)
    {}
    ~Determinant(){}

    #pragma mark Public member functions
    RealNumericValueType Value() const
    {
      if(this -> Order().first > 3 && this -> Order().second > 3)
        throw std::runtime_error("Determinant beyond order 3 has not been implemented yet");

      if(this -> Order().first == 1 && this -> Order().second == 1)
        return this -> operator()(0,0);
      else if(this -> Order().first == 2 && this -> Order().second == 2)
        return (this -> operator()(0,0) * this -> operator()(1,1)) - (this -> operator()(1,0) * this -> operator()(0,1));
      else
      {
        RealNumericValueType _temp_Value = 0;
        for(size_t _column_index = 0; _column_index < this -> Order().second; _column_index += 1)
        {
          _temp_Value += (this -> operator()(0,_column_index) * CoFactor_ByIndex(0,_column_index));
        }
        return _temp_Value;
      }
    }

    RealNumericValueType Minor_ByIndex(const size_t& _row_index, const size_t& _column_index) const
    {
      if(!(_row_index >= 0 && _row_index < this -> Order().first && _column_index >= 0 && _column_index < this -> Order().second))
        throw std::out_of_range("Out of range row or column index supplied");

      if(this -> Order().first > 3 && this -> Order().second > 3)
        throw std::runtime_error("Determinant beyond order 3 has not been implemented yet");
      if(this -> Order().first == 1 && this -> Order().second == 1)
        return this -> operator ()(0,0);
      else if(this -> Order().first == 2 && this -> Order().second == 2)
      {
        if(_row_index == _column_index)
        {
          if(_row_index == 0 && _column_index == 0) return this -> operator()(1,1);
          else return  this -> operator()(0,0);
        }else
        {
          if(_row_index == 0 && _column_index == 1) return this -> operator()(1,0);
          else return this -> operator()(0,1);
        }
      }
      else
      {
        std::vector<algebra::Row<RealNumericValueType>> _rows = this -> Rows();
        _rows.erase(_rows.begin() + _row_index);
        std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row) {
          _row.Erase_ByIndex(_column_index);
        });
        const algebra::Determinant<RealNumericValueType> _temp_Determinant = _rows;
        return _temp_Determinant.Value();
       }
    }

    RealNumericValueType CoFactor_ByIndex(const size_t& _row_index, const size_t& _column_index) const
    {
      if (IsEvenPower(_row_index,_column_index)) return Minor_ByIndex(_row_index,_column_index);
      else return -Minor_ByIndex(_row_index,_column_index);
      //RECON: Reconsider the logic here
      // return (RealNumericValueType) std::pow(-1,(_row_index + _column_index)) * ;
    }
  };

  double DeterminantUtility_AreaOfTriangle(const std::vector<std::pair<int,int>>& _points_of_triangle)
  {
    std::vector<int> _minor_x_buffer;
    _minor_x_buffer.reserve(3);
    std::vector<int> _minor_y_buffer;
    _minor_y_buffer.reserve(3);
    std::for_each(_points_of_triangle.begin(),_points_of_triangle.end(),[&](const std::pair<int,int>& _point) {
      _minor_x_buffer.emplace_back(_point.first);
      _minor_y_buffer.emplace_back(_point.second);
    });
    const std::vector<algebra::Column<int>> _major_column_buffer = {_minor_x_buffer,_minor_y_buffer,{1,1,1}};
    const algebra::Determinant<int> _temp_Determinant = _major_column_buffer;
    if(_temp_Determinant.Value() == 0)
      throw std::runtime_error("Collinear points are provided for computing area");
    else return std::fabs(0.5 * _temp_Determinant.Value());
  }
} // algebra




#endif //DETERMINANT_H
