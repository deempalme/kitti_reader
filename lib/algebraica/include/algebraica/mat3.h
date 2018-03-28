#ifndef ALGEBRAICA_MAT3_H
#define ALGEBRAICA_MAT3_H

#include "vec3.h"

#include <cmath>
#include <iostream>
#include <iomanip>
#include <assert.h>

#ifndef ALGTEM
#define ALGTEM template<typename T>
#endif

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | ------------------------------------ three-dimensional matrix --------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica{
  ALGTEM class mat3
  {
  public:
    mat3(T m00 = 1, T m01 = 0, T m02 = 0,
         T m10 = 0, T m11 = 1, T m12 = 0,
         T m20 = 0, T m21 = 0, T m22 = 1) :
      m_{vec3<T>(m00, m10, m20), vec3<T>(m01, m11, m21), vec3<T>(m02, m12, m22)} {}

    mat3(const mat3<T> &m){ *this = m; }
    /*
     * Converts this matrix into a identity
     */
    mat3<T>& to_identity();
    /*
     * Calculated the determinant
     */
    const T determinant();
    /*
     * returns the row element of this matrix
     */
    vec3<T>& operator[](const unsigned int index);
    /*
     * returns a constant of the row element of this matrix
     */
    const vec3<T>& operator[](const unsigned int index) const;
    /*
     * scalar-matrix assigment division
     *
     * @param {number} s is a scalar number
     */
    mat3<T>& operator/=(const T s);
    /*
     * value printing
     */
    friend std::ostream& operator<<(std::ostream &os, const mat3<T> &m){
      return os << "mat3(" << std::setprecision(3) << std::fixed
                << m[0][0] << ", " << m[1][0] << ", " << m[2][0]<< ",\n"
                << "     " << m[0][1] << ", " << m[1][1] << ", " << m[2][1] << ",\n"
                << "     " << m[0][2] << ", " << m[1][2] << ", " << m[2][2] << ")\n";
    }
    /*
     * Returns the data's address
     */
    T* data();
    /*
     * Returns the constant data's address
     */
    const T* data() const;

  private:
    union{
      vec3<T> m_[3];
      T data_[9];
    };
  };

  typedef mat3<double> mat3d;
  typedef mat3<float> mat3f;
  typedef mat3<int> mat3i;
}

#endif // ALGEBRAICA_MAT3_H
