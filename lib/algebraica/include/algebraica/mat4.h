#ifndef ALGEBRAICA_MAT4_H
#define ALGEBRAICA_MAT4_H

#include "vec3.h"
#include "vec4.h"
#include "mat3.h"
#include "quaternion.h"

#include <cmath>
#include <iostream>
#include <iomanip>

#ifndef ALGTEM
#define ALGTEM template<typename T>
#endif

/*
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
 | ------------------------------------ four-dimensional matrix ---------------------------------- |
 +–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––+
*/
namespace Algebraica {
  ALGTEM class mat4
  {
  public:
    mat4(T m00 = 1, T m01 = 0, T m02 = 0, T m03 = 0,
         T m10 = 0, T m11 = 1, T m12 = 0, T m13 = 0,
         T m20 = 0, T m21 = 0, T m22 = 1, T m23 = 0,
         T m30 = 0, T m31 = 0, T m32 = 0, T m33 = 1) :
      m_{vec4<T>(m00, m10, m20, m30),
         vec4<T>(m01, m11, m21, m31),
         vec4<T>(m02, m12, m22, m32),
         vec4<T>(m03, m13, m23, m33)}
    {}

    mat4(const vec4<T> column_0,
         const vec4<T> column_1,
         const vec4<T> column_2,
         const vec4<T> column_3) :
      m_{column_0, column_1, column_2, column_3}
    {}

    mat4(const mat4<T> &m) { *this = m; }
    /*
   * Converts this matrix into a identity
   */
    mat4<T>& to_identity();
    /*
   * Converts this matrix into a zero matrix
   */
    mat4<T>& to_zero();
    /*
   * Calculated the determinant
   */
    T determinant();
    const T determinant() const;
    /*
   * Rotates a mat4 by the given angle around the given axis
   *
   * @param {number} angle is the angle to rotate the matrix by
   * @param {vec3} v is a vector with the axes to rotate around
   */
    mat4<T>& rotate(const T angle, const vec3<T> &v);
    mat4<T>& rotate(const T pitch, const T yaw, const T roll);
    mat4<T>& from_euler(const T pitch, const T yaw, const T roll);
    mat4<T>& rotate(const T x, const T y, const T z, const T w);
    mat4<T>& rotate(const quaternion<T> quat);
    mat4<T>& from_quaternion(const T x, const T y, const T z, const T w);
    mat4<T>& from_quaternion(const quaternion<T> quat);
    mat4<T>& normalize_and_rotate(const T non_normalized_x, const T non_normalized_y,
                                  const T non_normalized_z, const T non_normalized_w);
    mat4<T>& normalize_and_rotate(const quaternion<T> non_normalized_quaternion);
    mat4<T>& from_non_normalized_quaternion(const T non_normalized_x, const T non_normalized_y,
                                            const T non_normalized_z, const T non_normalized_w);
    mat4<T>& from_non_normalized_quaternion(const quaternion<T> non_normalized_quaternion);
    /*
   * Rotates a matrix by the given angle around the X axis not using SIMD
   *
   * @param {Number} angle is the angle in radians to rotate the matrix by
   */
    mat4<T>& rotate_x(const T angle);
    /*
   * Rotates a matrix by the given angle around the Y axis not using SIMD
   *
   * @param {Number} angle is the angle in radians to rotate the matrix by
   */
    mat4<T>& rotate_y(const T angle);
    /*
   * Rotates a matrix by the given angle around the Z axis not using SIMD
   *
   * @param {Number} angle is the angle in radians to rotate the matrix by
   */
    mat4<T>& rotate_z(const T angle);
    /*
   * Translates this matrix by the given vector
   *
   * @param {vec3} v translation vector
   */
    mat4<T>& translate(const vec3<T> &v);
    /*
   * Translates this matrix by given X, Y and Z axes
   *
   * @param {number} x translation in X axis
   * @param {number} y translation in Y axis
   * @param {number} z translation in Z axis
   */
    mat4<T>& translate(const T x, const T y, const T z);
    /*
   * Scales all axes of this matrix by the given number
   *
   * @param {muber} s scaling factory
   */
    mat4<T>& scale(const T s);
    /*
   * Could scale all axes of this matrix by the given axes value inside the vector
   *
   * @param {vec3} v scaling factory for each axis
   */
    mat4<T>& scale(const vec3<T> &v);
    /*
   * Scale each axis by a given number
   *
   * @param {number} x scaling factory for X axis
   * @param {number} y scaling factory for Y axis
   * @param {number} z scaling factory for Z axis
   */
    mat4<T>& scale(const T x, const T y, const T z);
    /*
   * Calculates the normal matrix
   */
    mat3<T> normal_matrix();
    const mat3<T> normal_matrix() const;
    /*
   * Creates an orthogonal matrix
   *
   * @param {number} left is the left limit measured from center
   * @param {number} right is the right limit measured from center
   * @param {number} bottom is the bottom limit measured from center
   * @param {number} top is the left top measured from center
   * @param {number} near_plane indicates the minimum limit for the objects to appear in screen
   * @param {number} far_plane indicates the maximum limit for the objects to appear in screen
   */
    mat4<T>& ortho(const T left, const T right, const T bottom, const T top,
                   const T near_plane, const T far_plane);
    static mat4<T> from_ortho(const T left, const T right, const T bottom, const T top,
                              const T near_plane, const T far_plane);
    /*
   * Creates a perspective matrix
   *
   * @param {number} fov is the field of view in radians
   * @param {number} aspect is the aspect radio
   * @param {number} near indicates the minimum limit for the objects to appear in screen
   * @param {number} far indicates the maximum limit for the objects to appear in screen
   */
    mat4<T>& perspective(const T fov, const T aspect, const T near, const T far);
    /*
   * Pocisionates the camera in a 3D space
   *
   * @param {vec3} eye is the 3D position of the camera's lenses
   * @param {vec3} center is the 3D position of the point in which the camera is aimed to
   * @param {vec3} up is the camera's vertical position vector
   */
    mat4<T>& look_at(vec3<T> eye, vec3<T> center, vec3<T> up);

    static mat4<T> lookAt(vec3<T> eye, vec3<T> center, vec3<T> up);
    /*
   * Returns the same transformation matrix without the translation component
   */
    mat4<T> stationary();
    const mat4<T> stationary() const;
    /*
   * Returns the same transformation matrix without the rotation and scale components
   */
    mat4<T> only_translation();
    const mat4<T> only_translation() const;
    /*
   * Access the data of this matrix
   *
   * @param {unsigned int} index is the array index of this vector data, maximum: 3
   */
    vec4<T>& operator[](const int index);
    const vec4<T>& operator[](const int index) const;
    /*
   * Scalar-matrix addition
   *
   * @param {number} s is a scalar number
   */
    mat4<T> operator+(const T s);
    const mat4<T> operator+(const T s) const;
    /*
   * Matrix-matrix addition
   *
   * @param {mat4} m is the added matrix
   */
    mat4<T> operator+(const mat4<T> &m);
    const mat4<T> operator+(const mat4<T> &m) const;
    /*
   * Scalar-matrix substraction
   *
   * @param {number} s is a scalar number
   */
    mat4<T> operator-(const T s);
    const mat4<T> operator-(const T s) const;
    /*
   * Matrix-matrix substraction
   *
   * @param {mat4} m is the substracted matrix
   */
    mat4<T> operator-(const mat4<T> &m);
    const mat4<T> operator-(const mat4<T> &m) const;
    /*
   * Scalar-matrix multiplication
   *
   * @param {number} s is a scalar number
   */
    mat4<T> operator*(const T s);
    const mat4<T> operator*(const T s) const;
    /*
   * Matrix-matrix multiplication
   *
   * @param {mat4} m is the multiplier matrix
   */
    mat4<T> operator*(const mat4<T> &m);
    const mat4<T> operator*(const mat4<T> &m) const;
    /*
   * Vector-matrix multiplication
   *
   * @param {vec4} v is the multiplier vector
   */
    vec4<T> operator*(const vec4<T> &v);
    const vec4<T> operator*(const vec4<T> &v) const;
    /*
   * Vector-matrix multiplication
   *
   * @param {vec3} v is the multiplier vector
   */
    vec3<T> operator*(const vec3<T> &v);
    const vec3<T> operator*(const vec3<T> &v) const;
    /*
   * matrix-matrix assigment addition
   *
   * @param {mat4} m is the added matrix
   */
    mat4<T>& operator+=(const mat4<T> &m);
    /*
   * scalar-matrix assigment addition
   *
   * @param {number} s is a scalar number
   */
    mat4<T>& operator+=(const T s);
    /*
   * matrix-matrix assigment substraction
   *
   * @param {mat4} m is the substracted matrix
   */
    mat4<T>& operator-=(const mat4<T> &m);
    /*
   * scalar-matrix assigment substraction
   *
   * @param {number} s is a scalar number
   */
    mat4<T>& operator-=(const T s);
    /*
   * matrix-matrix assigment multiplication
   *
   * @param {mat4} m is the multiplied matrix
   */
    mat4<T>& operator*=(const mat4<T> &m);
    /*
   * scalar-matrix assigment multiplication
   *
   * @param {number} s is a scalar number
   */
    mat4<T>& operator*=(const T s);
    /*
   * scalar-matrix assigment division
   *
   * @param {number} s is a scalar number
   */
    mat4<T>& operator/=(const T s);
    /*
   * value printing
   */
    friend std::ostream& operator<<(std::ostream &os, const mat4<T> &m){
      return os << "mat4(" << std::setprecision(4) << std::fixed <<
                   m[0][0] << ", " << m[1][0] << ", " << m[2][0] << ", " << m[3][0] << ",\n     " <<
                   m[0][1] << ", " << m[1][1] << ", " << m[2][1] << ", " << m[3][1] << ",\n     " <<
                   m[0][2] << ", " << m[1][2] << ", " << m[2][2] << ", " << m[3][2] << ",\n     " <<
                   m[0][3] << ", " << m[1][3] << ", " << m[2][3] << ", " << m[3][3] << ")\n";
    }
    /*
   * pointer to raw data
   */
    T* data();
    /*
   * constant pointer to raw data
   */
    const T* data() const;

  private:
    union{
      vec4<T> m_[4];
      T data_[16];
    };
  };

  typedef mat4<double> mat4d;
  typedef mat4<float> mat4f;
  typedef mat4<int> mat4i;
}

#endif // ALGEBRAICA_MAT4_H
