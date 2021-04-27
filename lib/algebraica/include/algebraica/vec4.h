#ifndef ALGEBRAICA_VECTOR4_H
#define ALGEBRAICA_VECTOR4_H

#include "algebraica/vec2.h"
#include "algebraica/vec3.h"

#include <cmath>
#include <ostream>
#include <assert.h>

#ifndef ALGTEM
#define ALGTEM template<typename T>
#endif

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | ------------------------------------ four-dimensional vector ---------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica {
  ALGTEM class vec4
  {
  public:
    vec4(T x = 0, T y = 0, T z = 0, T w = 0) : values_{x, y, z, w} {}

    vec4(const vec4<T> &v){ *this = v; }

    vec4(const vec3<T> &v, const T w = 0) : values_{v[0], v[1], v[2], w} {}

    vec4(const vec2<T> &v) : values_{v[0], v[1], 0, 0} {}
    /*
   * Access the constant value of each axis
   *
   */
    const T x() const { return values_[0]; }
    const T y() const { return values_[1]; }
    const T z() const { return values_[2]; }
    const T w() const { return values_[3]; }
    /*
   * Access the value of each axis
   *
   * @param {unsigned int} index is the array's index
   */
    T& operator[](const unsigned int index);
    const T& operator[](const unsigned int index) const;
    /*
   * Sets all the values
   */
    vec4<T>& operator()(const T x = 0, const T y = 0, const T z = 0, const T w = 0);
    /*
   * Add a value to the vector
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T> operator+(const vec4<T> &v);
    const vec4<T> operator+(const vec4<T> &v) const;
    /*
   * @param {number} s is a scalar number
   */
    vec4<T> operator+(const T &s);
    const vec4<T> operator+(const T &s) const;
    /*
   * Substracts a value from the vector
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T> operator-(const vec4<T> &v);
    const vec4<T> operator-(const vec4<T> &v) const;
    /*
   * @param {number} s is a scalar number
   */
    vec4<T> operator-(const T &s);
    const vec4<T> operator-(const T &s) const;
    /*
   * Multiplies this vector for a value
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T> operator*(const vec4<T> &v);
    const vec4<T> operator*(const vec4<T> &v) const;
    /*
   * @param {vec3} v is a 3 dimensional vector
   */
    vec4<T> operator*(const vec3<T> &v);
    /*
   * @param {number} s is a scalar number
   */
    vec4<T> operator*(const T &s);
    const vec4<T> operator*(const T &s) const;
    /*
   * Divides this vector for a value
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T> operator/(const vec4<T> &v);
    const vec4<T> operator/(const vec4<T> &v) const;
    /*
   * @param {number} s is a scalar number
   */
    vec4<T> operator/(const T &s);
    const vec4<T> operator/(const T &s) const;
    /*
   * Vector assigment addition
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T>& operator+=(const vec4<T> &v);
    /*
   * @param {number} s is a scalar number
   */
    vec4<T>& operator+=(const T &s);
    /*
   * Vector assigment substraction
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T>& operator-=(const vec4<T> &v);
    /*
   * @param {number} s is a scalar number
   */
    vec4<T>& operator-=(const T &s);
    /*
   * Vector assigment multiplication
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T>& operator*=(const vec4<T> &v);
    /*
   * @param {number} s is a scalar number
   */
    vec4<T>& operator*=(const T &s);
    /*
   * Vector assigment division
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    vec4<T>& operator/=(const vec4<T> &v);
    /*
   * @param {number} s is a scalar number
   */
    vec4<T>& operator/=(const T &s);
    /*
   * Vector incremment prefix
   */
    vec4<T>& operator++();
    /*
   * Vector incremment suffix
   */
    vec4<T> operator++(int);
    /*
   * Vector decrement prefix
   */
    vec4<T>& operator--();
    /*
   * Vector decrement suffix
   */
    vec4<T> operator--(int);
    /*
   * Vector's printing output
   *
   * @param {ostream} os is the output stream reference from std::ostream
   */
    friend std::ostream& operator<<(std::ostream &os, const vec4<T> &v){
      return os << "vec4(" << v.x() << ", " << v.y() << ", " << v.z() << ", " << v.w() << ") ";
    }
    /*
   * Verifying if two vectors are equal
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator==(const vec4<T> &v);
    const bool operator==(const vec4<T> &v) const;
    /*
   * Verifying if two vectors are different
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator!=(const vec4<T> &v);
    const bool operator!=(const vec4<T> &v) const;
    /*
   * Vectors dimensional comparation (greater than)
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator>(const vec4<T> &v);
    const bool operator>(const vec4<T> &v) const;
    /*
   * Vectors dimensional comparation (lesser than)
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator<(const vec4<T> &v);
    const bool operator<(const vec4<T> &v) const;
    /*
   * Vectors dimensional comparation (greater or equal than)
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator>=(const vec4<T> &v);
    const bool operator>=(const vec4<T> &v) const;
    /*
   * Vectors dimensional comparation (lesser or equal than)
   *
   * @param {vec4} v is a 4 dimensional vector
   */
    bool operator<=(const vec4<T> &v);
    const bool operator<=(const vec4<T> &v) const;
    /*
   * pointer to this vector data
   */
    const T* data() const;
    T* data();
    /*
   * vector length
   */
    T length();
    const T length() const;
    /*
   * normalizing this vector
   */
    vec4<T>& normalize();
    /*
   * return the normalized vector
   */
    vec4<T> normalized();
    const vec4<T> normalized() const;
    /*
   * return the sum of each vector's element
   */
    T add();
    const T add() const;

  private:
    T values_[4];
  };

  typedef vec4<double> vec4d;
  typedef vec4<float> vec4f;
  typedef vec4<int> vec4i;
}

#endif // ALGEBRAICA_VECTOR4_H
