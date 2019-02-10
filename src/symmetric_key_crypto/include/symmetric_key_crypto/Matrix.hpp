#ifndef MATRIX_H
#define MATRIX_H

#include "MultiSequence.hpp"
#include "Determinant.hpp"

namespace algebra
{
  enum ExpansionType {E_AlongRow,E_AlongColumn};
  enum ContractionType {C_AlongRow,C_AlongColumn};
} // algebra


namespace algebra
{
  template<typename RealNumericValueType>
  class Matrix: public MultiSequence<RealNumericValueType>
  {
    static_assert(std::is_same<RealNumericValueType,int>::value ||
    std::is_same<RealNumericValueType,long>::value ||
    std::is_same<RealNumericValueType,float>::value ||
    std::is_same<RealNumericValueType,double>::value ||
    std::is_same<RealNumericValueType,algebra::ElementProtocol>::value,"Container can accept only real numbers for now.");

    #pragma mark Public constructors
    public:
    Matrix() = delete;
    Matrix(const std::initializer_list<std::vector<RealNumericValueType>>& _init_list)
    :MultiSequence<RealNumericValueType>(_init_list)
    {}
    Matrix(const std::vector<std::vector<RealNumericValueType>>& _vectors)
    :MultiSequence<RealNumericValueType>(_vectors)
    {}
    Matrix(const std::vector<algebra::Row<RealNumericValueType>>& _rows)
    :MultiSequence<RealNumericValueType>(_rows)
    {}
    Matrix(const std::vector<algebra::Column<RealNumericValueType>>& _columns)
    :MultiSequence<RealNumericValueType>(_columns)
    {}
    Matrix(const Matrix& _matrix)
    :MultiSequence<RealNumericValueType>(_matrix)
    {}
    ~Matrix(){}

    #pragma mark Public member functions
    public:
    inline bool IsRowMatrix() const { return this -> Order().first == 1; }

    inline bool IsColumnMatrix() const { return this -> Order().second == 1; }

    inline bool IsRectangularMatrix() const { return this -> Order().first != this -> Order().second; }

    inline bool IsSquareMatrix() const { return this -> Order().first == this -> Order().second; }

    const std::vector<RealNumericValueType> MainDiagonalElements() const
    {
      if(IsRectangularMatrix())
        throw std::logic_error("Only square matrices are having main diagonal elements");

      std::vector<size_t> _buffer(this -> Order().first);
      std::generate(_buffer.begin(),_buffer.end(),[_index = -1]() mutable {
        return _index += 1;
      });
      std::vector<RealNumericValueType> _diagonalElements(this -> Order().first);
      std::transform(_buffer.begin(),_buffer.end(),_diagonalElements.begin(),[&](const size_t& _index){
        return this -> operator()(_index,_index);
      });
      return _diagonalElements;
    }

    bool IsDiagonalMatrix() const
    {
      if(!IsSquareMatrix())
        return false;
      for(size_t row_index = 0; row_index < this -> Order().first; row_index += 1)
      {
        if(this -> operator()(row_index,row_index) == 0)
          return false;

        for(size_t column_index = row_index + 1; column_index < this -> Order().second; column_index += 1)
        {
          if(this -> operator()(row_index,column_index) != 0)
            return false;
          if(this -> operator()(column_index,row_index) != 0)
            return false;
        }
      }
      return true;
    }

    bool IsScalarMatrix() const
    {
      if(!IsDiagonalMatrix())
        return false;
      std::vector<RealNumericValueType> _mainDiagonalElements = MainDiagonalElements();
      return std::adjacent_find(_mainDiagonalElements.begin(),_mainDiagonalElements.end(),std::not_equal_to<RealNumericValueType>()) == _mainDiagonalElements.end();
    }

    bool IsIdentityMatrix() const
    {
      if(!IsScalarMatrix())
        return false;
      std::vector<RealNumericValueType> _mainDiagonalElements = MainDiagonalElements();
      if(_mainDiagonalElements[0] != 1)
        return false;
      return std::adjacent_find(_mainDiagonalElements.begin(),_mainDiagonalElements.end(),std::not_equal_to<RealNumericValueType>()) == _mainDiagonalElements.end();
    }

    bool IsNullMatrix() const
    {
      bool _isNull = true;
      std::for_each(this -> m_Container -> begin(),this -> m_Container -> end(),[&](const std::vector<RealNumericValueType>& _vector){
        if(_vector[0] != 0)
          _isNull = false;
        if(!(std::adjacent_find(_vector.begin(),_vector.end(),std::not_equal_to<RealNumericValueType>()) == _vector.end()))
          _isNull = false;
      });
      return _isNull;
    }

    bool IsUpperTriangularMatrix() const
    {
      if(!IsSquareMatrix())
        return false;
      for(size_t row_index = 0; row_index < this -> Order().first; row_index += 1)
      {
        if(this -> operator()(row_index,row_index) == 0)
          return false;

        for(size_t column_index = row_index + 1; column_index < this -> Order().second; column_index += 1)
        {
          if(this -> operator()(row_index,column_index) == 0)
            return false;
          if(this -> operator()(column_index,row_index) != 0)
            return false;
        }
      }
      return true;
    }

    bool IsLowerTriangularMatrix() const
    {
      if(!IsSquareMatrix())
        return false;
      for(size_t row_index = 0; row_index < this -> Order().first; row_index += 1)
      {
        if(this -> operator()(row_index,row_index) == 0)
          return false;

        for(size_t column_index = row_index + 1; column_index < this -> Order().second; column_index += 1)
        {
          if(this -> operator()(row_index,column_index) != 0)
            return false;
          if(this -> operator()(column_index,row_index) == 0)
            return false;
        }
      }
      return true;
    }

    bool IsTriangularMatrix() const
    {
      return IsUpperTriangularMatrix() || IsLowerTriangularMatrix();
    }

    RealNumericValueType Trace() const
    {
      if(!IsSquareMatrix())
        throw std::logic_error("Only square matrices can have trace value");

      const std::vector<RealNumericValueType> _mainDiagonalElements = MainDiagonalElements();
      const RealNumericValueType _sumOfDiagonalElements = std::accumulate(_mainDiagonalElements.begin(),_mainDiagonalElements.end(),0.0);
      return _sumOfDiagonalElements;
    }

    inline bool IsMultipliableWith(const algebra::Matrix<RealNumericValueType>& _matrix) const
    {
      return (this -> Order().second == _matrix.Order().first);
    }

    void For_EachRow(const std::function<void(const algebra::Row<RealNumericValueType>&)>& _lambda) const
    {
      std::vector<algebra::Row<RealNumericValueType>> _rows = this -> Rows();
      std::for_each(_rows.begin(),_rows.end(),[&](const algebra::Row<RealNumericValueType>& _row){
        _lambda(_row);
      });
    }

    void For_EachColumn(const std::function<void(const algebra::Column<RealNumericValueType>&)>& _lambda) const
    {
      std::vector<algebra::Column<RealNumericValueType>> _columns = this -> Columns();
      std::for_each(_columns.begin(),_columns.end(),[&](const algebra::Column<RealNumericValueType>& _column){
        _lambda(_column);
      });
    }

    Matrix<RealNumericValueType> Map(const std::function<RealNumericValueType(const RealNumericValueType&)>& _lambda) const
    {
      std::vector<std::vector<RealNumericValueType>> _mapped_Container;
      _mapped_Container.reserve(this -> Order().first);
      std::transform(this -> m_Container -> begin(), this -> m_Container -> end(),std::back_inserter(_mapped_Container),[&](const std::vector<RealNumericValueType>& _vector){
        std::vector<RealNumericValueType> _buffer;
        _buffer.reserve(this -> Order().second);
        std::transform(_vector.begin(),_vector.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element){
          return _lambda(_element);
        });
        return _buffer;
      });
      return algebra::Matrix<RealNumericValueType>(_mapped_Container);
    }

    Matrix<RealNumericValueType> Transpose() const
    {
      std::vector<algebra::Column<RealNumericValueType>> _columns;
      _columns.reserve(this -> Order().first);
      For_EachRow([&](const algebra::Row<RealNumericValueType>& _row){
        std::vector<RealNumericValueType> _buffer;
        _buffer.reserve(_row.Size());
        std::transform(_row.begin(),_row.end(),std::back_inserter(_buffer),[&](const RealNumericValueType& _element){
          return _element;
        });
        _columns.emplace_back(algebra::Column<RealNumericValueType>(_buffer));
      });
      return algebra::Matrix<RealNumericValueType>(_columns);
    }

    inline bool IsSymmetricMatrix() const
    {
      if(!IsSquareMatrix())
        return false;
      return *this == Transpose();
    }

    inline bool IsSkewSymmetricMatrix() const
    {
      if(!IsSquareMatrix())
        return false;
      const algebra::Matrix<RealNumericValueType> _matrix = Transpose().Map([&](const RealNumericValueType& _element){
        return -_element;
      });
      return *this == _matrix;
    }

    inline bool IsSingularMatrix() const
    {
      if(this -> Order().first > 3 || this -> Order().second > 3)
        throw std::runtime_error("SingularMatrix check beyond order 3 has not been implemented yet");
      if(!IsSquareMatrix()) return false;
      else
      {
        const std::vector<algebra::Row<RealNumericValueType>> _rows = this -> Rows();
        const algebra::Determinant<RealNumericValueType> _temp_Determinant = _rows;
        return _temp_Determinant.Value() == 0;
      }
    }

    inline bool IsInvertibleMatrix() const
    {
      if(this -> Order().first > 3 || this -> Order().second > 3)
        throw std::runtime_error("InvertibleMatrix check beyond order 3 has not been implemented yet");
      if(!IsSquareMatrix()) return false;
      const algebra::Determinant<RealNumericValueType> _temp_Determinant = this -> Rows();
      return _temp_Determinant.Value() != 0;
    }
  };

  #pragma mark Operator overloaded functions
  template<typename RealNumericValueType>
  algebra::Matrix<RealNumericValueType> operator +(const algebra::Matrix<RealNumericValueType>& _lhs, const algebra::Matrix<RealNumericValueType>& _rhs)
  {
    if (!(_lhs.Order() == _rhs.Order()))
      throw std::invalid_argument("Matrices of separate orders can not be added");

    const std::vector<algebra::Row<RealNumericValueType>> _lhs_Rows = _lhs.Rows();
    const std::vector<algebra::Row<RealNumericValueType>> _rhs_Rows = _rhs.Rows();

    std::vector<algebra::Row<RealNumericValueType>> _sumOfRows;
    _sumOfRows.reserve(_lhs_Rows.size());
    std::transform(_lhs_Rows.begin(),_lhs_Rows.end(),_rhs_Rows.begin(),std::back_inserter(_sumOfRows),[&](const algebra::Row<RealNumericValueType>& _lhs, const algebra::Row<RealNumericValueType>& _rhs){
      return _lhs + _rhs;
    });
    return algebra::Matrix<RealNumericValueType>(_sumOfRows);
  }

  template<typename RealNumericValueType>
  algebra::Matrix<RealNumericValueType> operator -(const algebra::Matrix<RealNumericValueType>& _lhs, const algebra::Matrix<RealNumericValueType>& _rhs)
  {
    if(!(_lhs.Order() == _rhs.Order()))
      throw std::invalid_argument("Matrices of separate orders can not be added");

    const std::vector<algebra::Row<RealNumericValueType>> _lhs_Rows = _lhs.Rows();
    const std::vector<algebra::Row<RealNumericValueType>> _rhs_Rows = _rhs.Rows();

    std::vector<algebra::Row<RealNumericValueType>> _differenceOfRows;
    _differenceOfRows.reserve(_lhs_Rows.size());
    std::transform(_lhs_Rows.begin(),_lhs_Rows.end(),_rhs_Rows.begin(),std::back_inserter(_differenceOfRows),[&](const algebra::Row<RealNumericValueType>& _lhs, const algebra::Row<RealNumericValueType>& _rhs){
      return _lhs - _rhs;
    });
    return algebra::Matrix<RealNumericValueType>(_differenceOfRows);
  }

  template<typename RealNumericValueType>
  algebra::Matrix<RealNumericValueType> operator *(const RealNumericValueType& _scalar, const algebra::Matrix<RealNumericValueType>& _matrix)
  {
    std::vector<algebra::Row<RealNumericValueType>> _matrix_Rows = _matrix.Rows();

    std::vector<algebra::Row<RealNumericValueType>> _product;
    _product.reserve(_matrix_Rows.size());
    std::transform(_matrix_Rows.begin(),_matrix_Rows.end(),std::back_inserter(_product),[&](const algebra::Row<RealNumericValueType>& _row) {
      return _scalar * _row;
    });
    return algebra::Matrix<RealNumericValueType>(_product);
  }

  template<typename RealNumericValueType>
  algebra::Matrix<RealNumericValueType> operator *(const algebra::Matrix<RealNumericValueType>& _matrix, const RealNumericValueType& _scalar)
  {
    return _scalar * _matrix;
  }

  template<typename RealNumericValueType>
  algebra::Matrix<RealNumericValueType> operator *(const algebra::Matrix<RealNumericValueType>& _lhs, const algebra::Matrix<RealNumericValueType>& _rhs)
  {
    if(!_lhs.IsMultipliableWith(_rhs))
      throw std::invalid_argument("Argument matrices violate the criterion for multiplication.");
    std::vector<algebra::Row<RealNumericValueType>> _lhs_Rows = _lhs.Rows();
    std::vector<algebra::Column<RealNumericValueType>> _rhs_Columns = _rhs.Columns();

    std::vector<std::vector<RealNumericValueType>> _resultant_Rows;
    _resultant_Rows.reserve(_lhs.Order().first);
    std::transform(_lhs_Rows.begin(),_lhs_Rows.end(),std::back_inserter(_resultant_Rows),[&](const algebra::Row<RealNumericValueType>& _row) {

      std::vector<RealNumericValueType> _buffer;
      _buffer.reserve(_rhs.Order().second);
      std::for_each(_rhs_Columns.begin(),_rhs_Columns.end(),[&](const algebra::Column<RealNumericValueType>& _column){
        _buffer.emplace_back(_row * _column);
      });
      return _buffer;
    });
    return algebra::Matrix<RealNumericValueType>(_resultant_Rows);
  }

  Matrix<int> Ints(const size_t& _row, const size_t& _column, const int& _number)
  {
    std::vector<std::vector<int>> _vectors;
    _vectors.reserve(_row);
    for(size_t _row_index = 0; _row_index < _row; _row_index += 1)
    {
      std::vector<int> _buffer(_column);
      std::generate(_buffer.begin(),_buffer.end(),[counter = _number]() {return counter;});
      _vectors.emplace_back(_buffer);
    }
    return algebra::Matrix<int>(_vectors);
  }

  Matrix<int> Zeros(const size_t& _row, const size_t& _column)
  {
    return algebra::Ints(_row,_column,0);
  }

  Matrix<int> Ones(const size_t& _row, const size_t& _column)
  {
    return algebra::Ints(_row,_column,1);
  }

  Matrix<int> Identity(const size_t& _order)
  {
    algebra::Matrix<int> _zeroes = algebra::Zeros(_order,_order);
    for(size_t _matrix_index = 0; _matrix_index < _order; _matrix_index += 1)
      _zeroes(_matrix_index,_matrix_index) = 1;
    return _zeroes;
  }

  template <typename RealNumericValueType>
  Matrix<RealNumericValueType> FindAdjointMatrixFor(const algebra::Matrix<RealNumericValueType>& _matrix)
  {
    if(!_matrix.IsSquareMatrix())
      throw std::invalid_argument("Adjoint can be obtained only for square matrices");
    if(_matrix.Order().first > 3)
      throw std::runtime_error("Adjoint for matrix beyond order 3 has not been implemented yet");

    const std::vector<algebra::Row<RealNumericValueType>> _rows = _matrix.Rows();
    const algebra::Determinant<RealNumericValueType> _temp_Determinant = _rows;

    std::vector<std::vector<RealNumericValueType>> _major_row_buffer;
    _major_row_buffer.reserve(_matrix.Order().first);
    for(size_t _row_index = 0; _row_index < _matrix.Order().first; _row_index++)
    {
      std::vector<RealNumericValueType> _minor_row_buffer;
      _minor_row_buffer.reserve(_matrix.Order().second);
      for(size_t _column_index = 0; _column_index < _matrix.Order().second; _column_index++)
      {
        _minor_row_buffer.emplace_back(_temp_Determinant.CoFactor_ByIndex(_row_index,_column_index));
      }
      _major_row_buffer.emplace_back(_minor_row_buffer);
    }
    return Matrix<RealNumericValueType>(_major_row_buffer).Transpose();
  }

  Matrix<int> Invert(const algebra::Matrix<int>& _invertibleMatrix)
  {
    if(_invertibleMatrix.Order().first > 3 || _invertibleMatrix.Order().second > 3)
      throw std::runtime_error("Invert utility for matrices beyond order 3 has not been implemented yet");
    if(!_invertibleMatrix.IsInvertibleMatrix())
      throw std::invalid_argument("Matrix is not invertible");

    const algebra::Determinant<int> _temp_Determinant = _invertibleMatrix.Rows();
    const int _determinant_Value = _temp_Determinant.Value();
    const algebra::Matrix<int> _temp_AdjointMatrix = algebra::FindAdjointMatrixFor(_invertibleMatrix);

    int _temp_Multiplier = _determinant_Value < 0 ? -1 : 1;
    return  _temp_Multiplier * _temp_AdjointMatrix;
  }

  template<typename RealNumericValueType>
  void MatrixToArray(const algebra::Matrix<RealNumericValueType>& _from_matrix,const algebra::ExpansionType& _expansion_type,RealNumericValueType *_to_array)
  {
    size_t _to_array_index = 0;
    if(_expansion_type == algebra::ExpansionType::E_AlongRow)
    {
      _from_matrix.For_EachRow([&](const algebra::Row<RealNumericValueType>& _row) {
        std::for_each(_row.begin(),_row.end(),[&](const RealNumericValueType& _element){
          *(_to_array + _to_array_index) = _element;
          _to_array_index += 1;
        });
      });
    }else
    {
      _from_matrix.For_EachColumn([&](const algebra::Column<RealNumericValueType>& _column) {
        std::for_each(_column.begin(),_column.end(),[&](const RealNumericValueType& _element){
          *(_to_array + _to_array_index) = _element;
          _to_array_index += 1;
        });
      });
    }
  }

  template<typename RealNumericValueType>
  Matrix<RealNumericValueType> ArrayToMatrix(const RealNumericValueType* _from_array, const size_t& _from_array_length, const algebra::ContractionType& _contraction_type,const std::pair<size_t,size_t>& _intended_matrix_order)
  {
    if((_intended_matrix_order.first * _intended_matrix_order.second) != _from_array_length)
      throw std::length_error("Provided array length is not compatible with the intended order of matrix");

    if(_contraction_type == algebra::ContractionType::C_AlongRow)
    {
      std::vector<algebra::Row<RealNumericValueType>> _major_buffer;
      _major_buffer.reserve(_intended_matrix_order.first);
      for(size_t _major_index = 0; _major_index < _from_array_length; _major_index += _intended_matrix_order.second)
      {
        const size_t _pivot_index = _major_index;
        std::vector<RealNumericValueType> _minor_buffer;
        _minor_buffer.reserve(_intended_matrix_order.second);

        for(size_t _minor_index = _major_index; _minor_index < _pivot_index+_intended_matrix_order.second; _minor_index+= 1)
        {
          _minor_buffer.emplace_back(*(_from_array + _minor_index));
        }

        const algebra::Row<RealNumericValueType> _row = _minor_buffer;
        _major_buffer.emplace_back(_row);
      }
      return Matrix<RealNumericValueType>(_major_buffer);
    }else
    {
      std::vector<algebra::Column<RealNumericValueType>> _major_buffer;
      _major_buffer.reserve(_intended_matrix_order.second);
      for(size_t _major_index = 0; _major_index < _from_array_length; _major_index += _intended_matrix_order.first)
      {
        const size_t _pivot_index = _major_index;
        std::vector<RealNumericValueType> _minor_buffer;
        _minor_buffer.reserve(_intended_matrix_order.first);

        for(size_t _minor_index = _major_index; _minor_index < _pivot_index+_intended_matrix_order.first; _minor_index+= 1)
        {
          _minor_buffer.emplace_back(*(_from_array + _minor_index));
        }

        const algebra::Column<RealNumericValueType> _column = _minor_buffer;
        _major_buffer.emplace_back(_column);
      }
      return Matrix<RealNumericValueType>(_major_buffer);
    }
  }

  template<typename RealNumericValueType>
  std::vector<RealNumericValueType> MatrixToVector(const algebra::Matrix<RealNumericValueType>& _from_matrix, const algebra::ExpansionType& _expansion_type)
  {
    if(_expansion_type == algebra::ExpansionType::E_AlongRow)
    {
      std::vector<RealNumericValueType> _major_element_buffer;
      _major_element_buffer.reserve(_from_matrix.Order().first * _from_matrix.Order().second);
      _from_matrix.For_EachRow([&](const algebra::Row<RealNumericValueType>& _row) {
        std::for_each(_row.begin(),_row.end(),[&](const RealNumericValueType& _element) {
          _major_element_buffer.emplace_back(_element);
        });
      });
      return _major_element_buffer;
    }else
    {
      std::vector<RealNumericValueType> _major_element_buffer;
      _major_element_buffer.reserve(_from_matrix.Order().first * _from_matrix.Order().second);
      _from_matrix.For_EachColumn([&](const algebra::Column<RealNumericValueType>& _column) {
        std::for_each(_column.begin(),_column.end(),[&](const RealNumericValueType& _element) {
          _major_element_buffer.emplace_back(_element);
        });
      });
      return _major_element_buffer;
    }
  }

  template<typename RealNumericValueType>
  Matrix<RealNumericValueType> VectorToMatrix(const std::vector<RealNumericValueType>& _from_vector, const algebra::ContractionType& _contraction_type, const std::pair<size_t,size_t>& _intended_matrix_order)
  {
    if((_intended_matrix_order.first * _intended_matrix_order.second) != _from_vector.size())
      throw std::length_error("Provided vector size is not compatible with the intended order of matrix");
    if(_contraction_type == algebra::ContractionType::C_AlongRow)
    {
      std::vector<algebra::Row<RealNumericValueType>> _major_buffer;
      _major_buffer.reserve(_intended_matrix_order.first);
      for(size_t _major_index = 0; _major_index < _from_vector.size(); _major_index += _intended_matrix_order.second)
      {
        const size_t _pivot_index = _major_index;
        std::vector<RealNumericValueType> _minor_buffer;
        _minor_buffer.reserve(_intended_matrix_order.second);

        for(size_t _minor_index = _major_index; _minor_index < _pivot_index+_intended_matrix_order.second; _minor_index+= 1)
        {
          _minor_buffer.emplace_back(_from_vector[_minor_index]);
        }

        const algebra::Row<RealNumericValueType> _row = _minor_buffer;
        _major_buffer.emplace_back(_row);
      }
      return Matrix<RealNumericValueType>(_major_buffer);
    }else
    {
      std::vector<algebra::Column<RealNumericValueType>> _major_buffer;
      _major_buffer.reserve(_intended_matrix_order.second);
      for(size_t _major_index = 0; _major_index < _from_vector.size(); _major_index += _intended_matrix_order.first)
      {
        const size_t _pivot_index = _major_index;
        std::vector<RealNumericValueType> _minor_buffer;
        _minor_buffer.reserve(_intended_matrix_order.first);

        for(size_t _minor_index = _major_index; _minor_index < _pivot_index+_intended_matrix_order.first; _minor_index+= 1)
        {
          _minor_buffer.emplace_back(_from_vector[_minor_index]);
        }

        const algebra::Column<RealNumericValueType> _column = _minor_buffer;
        _major_buffer.emplace_back(_column);
      }
      return Matrix<RealNumericValueType>(_major_buffer);
    }
  }
} // algebra

#endif //MATRIX_H
