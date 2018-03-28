
#include "algebraica/mat3.h"

namespace Algebraica{
  ALGTEM mat3<T>& mat3<T>::to_identity(){
    m_[0][0] = 1, m_[0][1] = 0, m_[0][2] = 0;
    m_[1][0] = 0, m_[1][1] = 1, m_[1][2] = 0;
    m_[2][0] = 0, m_[2][1] = 0, m_[2][2] = 1;
  }

  ALGTEM const T mat3<T>::determinant(){
    return m_[0][0] * (m_[1][1] * m_[2][2] - m_[1][2] * m_[2][1])
        - m_[0][1] * (m_[1][0] * m_[2][2] - m_[1][2] * m_[2][0])
        + m_[0][2] * (m_[1][0] * m_[2][1] - m_[1][1] * m_[2][0]);;
  }

  ALGTEM vec3<T>& mat3<T>::operator[](const unsigned int index){
    assert(index >= 0 && index < 3);
    return m_[index];
  }

  ALGTEM const vec3<T>& mat3<T>::operator[](const unsigned int index) const{
    assert(index >= 0 && index < 3);
    return m_[index];
  }

  ALGTEM mat3<T>& mat3<T>::operator/=(const T s){
    m_[0] /= s;
    m_[1] /= s;
    m_[2] /= s;

    return *this;
  }

  ALGTEM T* mat3<T>::data(){
    return &data_[0];
  }

  ALGTEM const T* mat3<T>::data() const{
    return &data_[0];
  }

  template class mat3<double>;
  template class mat3<float>;
  template class mat3<int>;
}
