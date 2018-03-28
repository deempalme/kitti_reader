#ifndef ALGEBRAICA_VECTOR2_H
#define ALGEBRAICA_VECTOR2_H

#include <cmath>
#include <ostream>

#ifndef ALGTEM
#define ALGTEM template<typename T>
#endif

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | ------------------------------------ two-dimensional vector ----------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica {
  ALGTEM class vec2
  {
  public:
    vec2(T _x = 0, T _y = 0) : values_{_x, _y} {}
    vec2(const vec2<T> &v){ *this = v; }

    const T x() const { return values_[0]; }
    const T y() const { return values_[1]; }

    // index array access
    T& operator[](const unsigned int index);
    const T& operator[](const unsigned int index) const;
    // vector addition
    vec2<T> operator+(const vec2<T> &v);
    const vec2<T> operator+(const vec2<T> &v) const;
    // vector substraction
    vec2<T> operator-(vec2<T> &v);
    const vec2<T> operator-(vec2<T> &v) const;
    // vector multiplication
    vec2<T> operator*(const vec2<T> &v);
    const vec2<T> operator*(const vec2<T> &v) const;
    // vector division
    vec2<T> operator/(const vec2<T> &v);
    const vec2<T> operator/(const vec2<T> &v) const;
    // vector assigment addition
    vec2<T>& operator+=(const vec2<T> &v);
    // vector assigment substraction
    vec2<T>& operator-=(const vec2<T> &v);
    // vector assigment multiplication
    vec2<T>& operator*=(const vec2<T> &v);
    // vector assigment division
    vec2<T>& operator/=(const vec2<T> &v);
    // vector increment prefix
    vec2<T>& operator++();
    // vector increment postfix
    vec2<T> operator++(int);
    // vector decrement prefix
    vec2<T>& operator--();
    // operator overload for ()
    vec2<T>& operator ()(const T x = 0, const T y = 0);
    // vector decrement postfix
    vec2<T> operator--(int);
    // printing the values
    friend std::ostream& operator<<(std::ostream &os, const vec2<T> &v){
      return os << "vec2(" << v.x() << ", " << v.y() << ") ";
    }
    // verifying if two vectors are equal
    bool operator==(const vec2<T> &v);
    const bool operator==(const vec2<T> &v) const;
    // verifying if two vectors are different
    bool operator!=(const vec2<T> &v);
    const bool operator!=(const vec2<T> &v) const;
    // vectors dimensional comparation (greater than)
    bool operator>(const vec2<T> &v);
    const bool operator>(const vec2<T> &v) const;
    // vectors dimensional comparation (lesser than)
    bool operator<(const vec2<T> &v);
    const bool operator<(const vec2<T> &v) const;
    // vectors dimensional comparation (greater or equal than)
    bool operator>=(const vec2<T> &v);
    const bool operator>=(const vec2<T> &v) const;
    // vectors dimensional comparation (lesser or equal than)
    bool operator<=(const vec2<T> &v);
    const bool operator<=(const vec2<T> &v) const;
    // pointer to vector data
    T* data();
    // constant pointer to vector data
    const T* data() const;
    // vector length
    T length();
    const T length() const;
    // normalize this vector
    vec2<T>& normalize();
    // normalized vector
    vec2<T> normalized();
    const vec2<T> normalized() const;
    // rotate the vector through X axis
    vec2<T>& rotate(const T _angle);

  private:
    T values_[2];
  };

  typedef vec2<double> vec2d;
  typedef vec2<float> vec2f;
  typedef vec2<int> vec2i;

}

#endif // ALGEBRAICA_VECTOR2_H
