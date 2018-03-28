#ifndef ALGEBRAICA_VECTOR3_H
#define ALGEBRAICA_VECTOR3_H

#include <cmath>
#include <ostream>
#include "assert.h"

#ifndef ALGTEM
#define ALGTEM template<typename T>
#endif

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | ----------------------------------- three-dimensional vector ---------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica {
  ALGTEM class quaternion;

  ALGTEM class vec3
  {
  public:
    vec3(T x = 0, T y = 0, T z = 0) : values_{x, y, z} {}
    vec3(const vec3<T> &v){ *this = v; }

    const T x() const { return values_[0]; }
    const T y() const { return values_[1]; }
    const T z() const { return values_[2]; }

    /*
     * index array access
     */
    T& operator[](const unsigned int index);
    /*
     * constat index array access
     */
    const T& operator[](const unsigned int index) const;
    // vector addition
    vec3<T> operator+(const vec3<T> &v);
    const vec3<T> operator+(const vec3<T> &v) const;
    // vector substraction
    vec3<T> operator-(const vec3<T> &v);
    const vec3<T> operator-(const vec3<T> &v) const;
    // vector multiplication
    vec3<T> operator*(const vec3<T> &v);
    const vec3<T> operator*(const vec3<T> &v) const;
    vec3<T> operator*(const T s);
    const vec3<T> operator*(const T s) const;
    // vector division
    vec3<T> operator/(const vec3<T> &v);
    const vec3<T> operator/(const vec3<T> &v) const;
    vec3<T> operator/(const T s);
    const vec3<T> operator/(const T s) const;
    // vector assigment addition
    vec3<T>& operator+=(const vec3<T> &v);
    // vector assigment substraction
    vec3<T>& operator-=(const vec3<T> &v);
    // vector assigment multiplication
    vec3<T>& operator*=(const vec3<T> &v);
    // vector assigment multiplication
    vec3<T>& operator*=(const T s);
    // vector assigment division
    vec3<T>& operator/=(const vec3<T> &v);
    // vector assigment division
    vec3<T>& operator/=(const T s);
    // vector increment prefix
    vec3<T>& operator++();
    // vector increment postfix
    vec3<T> operator++(int);
    // vector decrement prefix
    vec3<T>& operator--();
    // vector decrement postfix
    vec3<T> operator--(int);
    vec3<T>& operator ()(const T x = 0, const T y = 0, const T z = 0);
    // printing the values
    friend std::ostream& operator<<(std::ostream &os, const vec3<T> &v){
      return os << "vec3(" << v.x() << ", " << v.y() << ", " << v.z() << ") ";
    }
    // verifying if two vectors are equal
    bool operator==(const vec3<T> &v);
    const bool operator==(const vec3<T> &v) const;
    // verifying if two vectors are different
    bool operator!=(const vec3<T> &v);
    const bool operator!=(const vec3<T> &v) const;
    // vectors dimensional comparation (greater than)
    bool operator>(const vec3<T> &v);
    const bool operator>(const vec3<T> &v) const;
    // vectors dimensional comparation (lesser than)
    bool operator<(const vec3<T> &v);
    const bool operator<(const vec3<T> &v) const;
    // vectors dimensional comparation (greater or equal than)
    bool operator>=(const vec3<T> &v);
    const bool operator>=(const vec3<T> &v) const;
    // vectors dimensional comparation (lesser or equal than)
    bool operator<=(const vec3<T> &v);
    const bool operator<=(const vec3<T> &v) const;
    // pointer to vector data
    T* data();
    // constant pointer to vector data
    const T* data() const;
    // vector length
    T length();
    const T length() const;
    // distance between two vectors
    static T distance(const vec3<T> &first, const vec3<T> &second);
    // normalize this vector
    vec3<T>& normalize();
    static vec3<T> normalize(vec3<T> v);
    // normalized vector
    vec3<T> normalized();
    const vec3<T> normalized() const;
    // cross product between two vectors
    static vec3<T> cross(const vec3<T> &u, const vec3<T> &v);
    // compute the dot product of two vectors
    static T dot(vec3<T> &u, const vec3<T> &v);
    // rotate the vector through X axis
    vec3<T>& rotateX(const T _angle);
    // rotate the vector through Y axis
    vec3<T>& rotateY(const T _angle);
    // rotate the vector through Z axis
    vec3<T>& rotateZ(const T _angle);
    // rotate using quaternion
    vec3<T>& rotate(const quaternion<T> quat);

  private:
    T values_[3];
  };

  typedef vec3<double> vec3d;
  typedef vec3<float> vec3f;
  typedef vec3<int> vec3i;
}

#endif // ALGEBRAICA_VECTOR3_H
