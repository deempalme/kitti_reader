
#include "algebraica/quaternion.h"

namespace Algebraica {
  ALGTEM T& quaternion<T>::operator[](const unsigned int index){
    assert(index >= 0 && index < 4);
    return values_[index];
  }

  ALGTEM const T& quaternion<T>::operator[](const unsigned int index) const{
    assert(index >= 0 && index < 4);
    return values_[index];
  }

  ALGTEM quaternion<T>& quaternion<T>::operator()(const T x, const T y, const T z, const T w){
    values_[0] = x;
    values_[1] = y;
    values_[2] = z;
    values_[3] = w;
    return *this;
  }

  ALGTEM quaternion<T>& quaternion<T>::from_euler(const T pitch, const T yaw, const T roll){
    const T two{T(2)};
    const vec3<T> c(std::cos(yaw / two), std::cos(roll / two), std::cos(pitch / two));
    const vec3<T> s(std::sin(yaw / two), std::sin(roll / two), std::sin(pitch / two));

    values_[0] = s[0] * c[1] * c[2] - c[0] * s[1] * s[2];
    values_[1] = c[0] * s[1] * c[2] + s[0] * c[1] * s[2];
    values_[2] = c[0] * c[1] * s[2] - s[0] * s[1] * c[2];
    values_[3] = c[0] * c[1] * c[2] + s[0] * s[1] * s[2];

    return *this;
  }

  ALGTEM quaternion<T> quaternion<T>::euler_to_quaternion(const T pitch, const T yaw, const T roll){
    quaternion<T> quat;
    const T two{T(2)};
    const vec3<T> c(std::cos(yaw / two), std::cos(roll / two), std::cos(pitch / two));
    const vec3<T> s(std::sin(yaw / two), std::sin(roll / two), std::sin(pitch / two));

    quat[0] = s[0] * c[1] * c[2] - c[0] * s[1] * s[2];
    quat[1] = c[0] * s[1] * c[2] + s[0] * c[1] * s[2];
    quat[2] = c[0] * c[1] * s[2] - s[0] * s[1] * c[2];
    quat[3] = c[0] * c[1] * c[2] + s[0] * s[1] * s[2];

    return quat;
  }

  ALGTEM vec3<T> quaternion<T>::to_euler(){
    vec3<T> euler;
    const T one{1}, two{2};
    const T sinr{two * (values_[3] * values_[0] + values_[1] * values_[2])};
    const T cosr{one - two * (values_[0] * values_[0] + values_[1] * values_[1])};
    euler[2] = std::atan2(sinr, cosr);

    const T sinp{two * (values_[3] * values_[1] - values_[2] * values_[0])};
    if (std::fabs(sinp) >= one)
      euler[0] = std::copysign(M_PI / 2.0, sinp);
    else
      euler[0] = std::asin(sinp);

    const T siny{two * (values_[3] * values_[2] + values_[0] * values_[1])};
    const T cosy{one - two * (values_[1] * values_[1] + values_[2] * values_[2])};
    euler[1] = std::atan2(siny, cosy);

    return euler;
  }

  ALGTEM const vec3<T> quaternion<T>::to_euler() const{
    vec3<T> euler;
    const T one{1}, two{2};
    const T sinr{two * (values_[3] * values_[0] + values_[1] * values_[2])};
    const T cosr{one - two * (values_[0] * values_[0] + values_[1] * values_[1])};
    euler[2] = std::atan2(sinr, cosr);

    const T sinp{two * (values_[3] * values_[1] - values_[2] * values_[0])};
    if (std::fabs(sinp) >= one)
      euler[0] = std::copysign(M_PI / 2.0, sinp);
    else
      euler[0] = std::asin(sinp);

    const T siny{two * (values_[3] * values_[2] + values_[0] * values_[1])};
    const T cosy{one - two * (values_[1] * values_[1] + values_[2] * values_[2])};
    euler[1] = std::atan2(siny, cosy);

    return euler;
  }

  ALGTEM void quaternion<T>::to_euler(T *pitch, T *yaw, T *roll){
    const T one{1}, two{2};
    const T sinr{two * (values_[3] * values_[0] + values_[1] * values_[2])};
    const T cosr{one - two * (values_[0] * values_[0] + values_[1] * values_[1])};
    *roll = std::atan2(sinr, cosr);

    const T sinp{two * (values_[3] * values_[1] - values_[2] * values_[0])};
    if (std::fabs(sinp) >= one)
      *pitch = std::copysign(M_PI / 2.0, sinp);
    else
      *pitch = std::asin(sinp);

    const T siny{two * (values_[3] * values_[2] + values_[0] * values_[1])};
    const T cosy{one - two * (values_[1] * values_[1] + values_[2] * values_[2])};
    *yaw = std::atan2(siny, cosy);
  }

  ALGTEM void quaternion<T>::to_euler(T *pitch, T *yaw, T *roll) const{
    const T one{1}, two{2};
    const T sinr{two * (values_[3] * values_[0] + values_[1] * values_[2])};
    const T cosr{one - two * (values_[0] * values_[0] + values_[1] * values_[1])};
    *roll = std::atan2(sinr, cosr);

    const T sinp{two * (values_[3] * values_[1] - values_[2] * values_[0])};
    if (std::fabs(sinp) >= one)
      *pitch = std::copysign(M_PI / 2.0, sinp);
    else
      *pitch = std::asin(sinp);

    const T siny{two * (values_[3] * values_[2] + values_[0] * values_[1])};
    const T cosy{one - two * (values_[1] * values_[1] + values_[2] * values_[2])};
    *yaw = std::atan2(siny, cosy);
  }

  ALGTEM vec3<T> quaternion<T>::to_3D(){
    return vec3<T>(values_[0], values_[1], values_[2]);
  }

  ALGTEM const vec3<T> quaternion<T>::to_3D() const{
    return vec3<T>(values_[0], values_[1], values_[2]);
  }

  ALGTEM T quaternion<T>::angle(){
    return T(std::acos(values_[3])) * T(2);
  }

  ALGTEM const T quaternion<T>::angle() const{
    return T(std::acos(values_[3])) * T(2);
  }

  ALGTEM T quaternion<T>::pitch(){
    const T two(2), one(1);
    const T sinp(two * (values_[3] * values_[1] - values_[2] * values_[0]));
//    if(std::fabs(sinp) >= one)
//      return T(std::copysign(M_PI_2, sinp));
//    else
      return T(std::asin(sinp));
  }

  ALGTEM const T quaternion<T>::pitch() const{
    const T two(2), one(1);
//    const T sinp(two * (values_[3] * values_[1] - values_[2] * values_[0]));
//    if(std::fabs(sinp) >= one)
//      return T(std::copysign(M_PI_2, sinp));
//    else
//      return T(std::asin(sinp));
    return T(std::atan2(two * (values_[0] * values_[3] + values_[1] * values_[2]),
                        one - two * (values_[0] * values_[0] + values_[1] * values_[1])));
  }

  ALGTEM T quaternion<T>::yaw(){
    const T two(2), one(1);
    return T(std::atan2(two * (values_[3] * values_[2] + values_[0] * values_[1]),
                        one - two * (values_[1] * values_[1] + values_[2] * values_[2])));
  }

  ALGTEM const T quaternion<T>::yaw() const{
    const T two(2), one(1);
    return T(std::atan2(two * (values_[3] * values_[2] + values_[0] * values_[1]),
                        one - two * (values_[1] * values_[1] + values_[2] * values_[2])));
  }

  ALGTEM T quaternion<T>::roll(){
    const T two(2), one(1);
    return T(std::atan2(two * (values_[3] * values_[0] + values_[1] * values_[2]),
                        one - two * (values_[0] * values_[0] + values_[1] * values_[1])));
  }

  ALGTEM const T quaternion<T>::roll() const{
    const T two(2), one(1);
    return T(std::atan2(two * (values_[3] * values_[0] + values_[1] * values_[2]),
                        one - two * (values_[0] * values_[0] + values_[1] * values_[1])));
  }

  ALGTEM quaternion<T>& quaternion<T>::rotate(const vec3<T> axis, const T angle){
    const T a(angle/2.0);
    const T s(std::sin(a));

    const quaternion<T> rot(axis[0] * s, axis[1] * s, axis[2] * s, T(std::cos(a)));
    return *this *= rot;
  }

  ALGTEM quaternion<T>& quaternion<T>::rotate(const T x, const T y, const T z, const T angle){
    const T a(angle/2.0);
    const T s(std::sin(a));

    const quaternion<T> rot(x * s, y * s, z * s, T(std::cos(a)));
    return *this *= rot;
  }

  ALGTEM quaternion<T> quaternion<T>::from_axis_and_angle(const vec3<T> axis, const T angle){
    const T a(angle/2);
    const T s(std::sin(a));

    return quaternion<T>(axis[0] * s, axis[1] * s, axis[2] * s, T(std::cos(a)));
  }

  ALGTEM quaternion<T> quaternion<T>::from_axis_and_angle(const T x, const T y, const T z,
                                                          const T angle){
    const T a(angle/2);
    const T s(std::sin(a));

    return quaternion<T>(x * s, y * s, z * s, T(std::cos(a)));
  }

  ALGTEM quaternion<T> quaternion<T>::operator/(const T &s){
    return quaternion<T>(values_[0] / s, values_[1] / s, values_[2] / s, values_[3] / s);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator/(const T &s) const{
    return quaternion<T>(values_[0] / s, values_[1] / s, values_[2] / s, values_[3] / s);
  }

  ALGTEM quaternion<T> quaternion<T>::operator*(const T &s){
    return quaternion<T>(values_[0] * s, values_[1] * s, values_[2] * s, values_[3] * s);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator*(const T &s) const{
    return quaternion<T>(values_[0] * s, values_[1] * s, values_[2] * s, values_[3] * s);
  }

  ALGTEM quaternion<T> quaternion<T>::operator*(const quaternion<T> &q){
    quaternion<T> result;
    result[0] =  values_[0] * q[3] + values_[1] * q[2] - values_[2] * q[1] + values_[3] * q[0];
    result[1] = -values_[0] * q[2] + values_[1] * q[3] + values_[2] * q[0] + values_[3] * q[1];
    result[2] =  values_[0] * q[1] - values_[1] * q[0] + values_[2] * q[3] + values_[3] * q[2];
    result[3] = -values_[0] * q[0] - values_[1] * q[1] - values_[2] * q[2] + values_[3] * q[3];

    return result;
  }

  ALGTEM const quaternion<T> quaternion<T>::operator*(const quaternion<T> &q) const{
    quaternion<T> result;
    result[0] =  values_[0] * q[3] + values_[1] * q[2] - values_[2] * q[1] + values_[3] * q[0];
    result[1] = -values_[0] * q[2] + values_[1] * q[3] + values_[2] * q[0] + values_[3] * q[1];
    result[2] =  values_[0] * q[1] - values_[1] * q[0] + values_[2] * q[3] + values_[3] * q[2];
    result[3] = -values_[0] * q[0] - values_[1] * q[1] - values_[2] * q[2] + values_[3] * q[3];

    return result;
  }
/*
  ALGTEM vec3<T> quaternion<T>::operator*(const vec3<T> &v){
    const vec3<T> Q{values_[0], values_[1], values_[2]};
    const vec3<T> A{vec3<T>::cross(v, Q) + v * values_[3]};

    return vec3<T>::cross(A, Q) * T(2);
  }
*/
  ALGTEM vec3<T> quaternion<T>::operator*(const vec3<T> &v){
//    const vec
    const vec3<T> Q{values_[0], values_[1], values_[2]};
    const vec3<T> A{vec3<T>::cross(v, Q) + v * values_[3]};

    return v + vec3<T>::cross(A, Q) * T(2);
  }

  ALGTEM const vec3<T> quaternion<T>::operator*(const vec3<T> &v) const{
    const vec3<T> Q{values_[0], values_[1], values_[2]};
    const vec3<T> A{vec3<T>::cross(v, Q) + v * values_[3]};

    return v + vec3<T>::cross(A, Q) * T(2);
  }

  ALGTEM quaternion<T> quaternion<T>::operator-(const T &s){
    return quaternion<T>(values_[0] - s, values_[1] - s, values_[2] - s, values_[3] - s);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator-(const T &s) const{
    return quaternion<T>(values_[0] - s, values_[1] - s, values_[2] - s, values_[3] - s);
  }

  ALGTEM quaternion<T> quaternion<T>::operator-(const quaternion<T> &q){
    return quaternion<T>(values_[0] - q[0], values_[1] - q[1], values_[2] - q[2], values_[3] - q[3]);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator-(const quaternion<T> &q) const{
    return quaternion<T>(values_[0] - q[0], values_[1] - q[1], values_[2] - q[2], values_[3] - q[3]);
  }

  ALGTEM quaternion<T> quaternion<T>::operator+(const T &s){
    return quaternion<T>(values_[0] + s, values_[1] + s, values_[2] + s, values_[3] + s);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator+(const T &s) const{
    return quaternion<T>(values_[0] + s, values_[1] + s, values_[2] + s, values_[3] + s);
  }

  ALGTEM quaternion<T> quaternion<T>::operator+(const quaternion<T> &q){
    return quaternion<T>(values_[0] + q[0], values_[1] + q[1],
        values_[2] + q[2], values_[3] + q[3]);
  }

  ALGTEM const quaternion<T> quaternion<T>::operator+(const quaternion<T> &q) const{
    return quaternion<T>(values_[0] + q[0], values_[1] + q[1],
        values_[2] + q[2], values_[3] + q[3]);
  }

  ALGTEM quaternion<T> &quaternion<T>::operator/=(const T &s){
    values_[0] /= s;
    values_[1] /= s;
    values_[2] /= s;
    values_[3] /= s;
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator*=(const quaternion<T> &q){
    quaternion<T> value(*this);
    values_[0] =  value[0] * q[3] + value[1] * q[2] - value[2] * q[1] + value[3] * q[0];
    values_[1] = -value[0] * q[2] + value[1] * q[3] + value[2] * q[0] + value[3] * q[1];
    values_[2] =  value[0] * q[1] - value[1] * q[0] + value[2] * q[3] + value[3] * q[2];
    values_[3] = -value[0] * q[0] - value[1] * q[1] - value[2] * q[2] + value[3] * q[3];
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator*=(const T &s){
    values_[0] *= s;
    values_[1] *= s;
    values_[2] *= s;
    values_[3] *= s;
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator-=(const quaternion<T> &q){
    values_[0] -= q[0];
    values_[1] -= q[1];
    values_[2] -= q[2];
    values_[3] -= q[3];
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator-=(const T &s){
    values_[0] -= s;
    values_[1] -= s;
    values_[2] -= s;
    values_[3] -= s;
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator+=(const quaternion<T> &q){
    values_[0] += q[0];
    values_[1] += q[1];
    values_[2] += q[2];
    values_[3] += q[3];
    return *this;
  }

  ALGTEM quaternion<T> &quaternion<T>::operator+=(const T &s){
    values_[0] += s;
    values_[1] += s;
    values_[2] += s;
    values_[3] += s;
    return *this;
  }

  ALGTEM bool quaternion<T>::operator==(const quaternion<T> &q){
    return values_[0] == q[0] && values_[1] == q[1] && values_[2] == q[2] && values_[3] == q[3];
  }

  ALGTEM const bool quaternion<T>::operator==(const quaternion<T> &q) const{
    return values_[0] == q[0] && values_[1] == q[1] && values_[2] == q[2] && values_[3] == q[3];
  }

  ALGTEM bool quaternion<T>::operator!=(const quaternion<T> &q){
    return !(values_[0] == q[0] && values_[1] == q[1] && values_[2] == q[2] && values_[3] == q[3]);
  }

  ALGTEM const bool quaternion<T>::operator!=(const quaternion<T> &q) const{
    return !(values_[0] == q[0] && values_[1] == q[1] && values_[2] == q[2] && values_[3] == q[3]);
  }

  ALGTEM bool quaternion<T>::operator>(const quaternion<T> &q){
    return length() > q.length();
  }

  ALGTEM const bool quaternion<T>::operator>(const quaternion<T> &q) const{
    return length() > q.length();
  }

  ALGTEM bool quaternion<T>::operator<(const quaternion<T> &q){
    return length() < q.length();
  }

  ALGTEM const bool quaternion<T>::operator<(const quaternion<T> &q) const{
    return length() < q.length();
  }

  ALGTEM bool quaternion<T>::operator>=(const quaternion<T> &q){
    return length() >= q.length();
  }

  ALGTEM const bool quaternion<T>::operator>=(const quaternion<T> &q) const{
    return length() >= q.length();
  }

  ALGTEM bool quaternion<T>::operator<=(const quaternion<T> &q){
    return length() <= q.length();
  }

  ALGTEM const bool quaternion<T>::operator<=(const quaternion<T> &q) const{
    return length() <= q.length();
  }

  ALGTEM T *quaternion<T>::data(){
    return &values_[0];
  }

  ALGTEM const T *quaternion<T>::data() const{
    return &values_[0];
  }

  ALGTEM quaternion<T> &quaternion<T>::normalize(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    values_[0] *= d;
    values_[1] *= d;
    values_[2] *= d;
    values_[3] *= d;
    return *this;
  }

  ALGTEM quaternion<T> quaternion<T>::normalized(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    return quaternion<T>(values_[0] * d, values_[1] * d, values_[2] * d, values_[3] * d);
  }

  ALGTEM const quaternion<T> quaternion<T>::normalized() const{
    T d(length());
    d = (d > 0)? 1/d : 0;
    return quaternion<T>(values_[0] * d, values_[1] * d, values_[2] * d, values_[3] * d);
  }

  ALGTEM T quaternion<T>::length(){
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] +
        values_[2] * values_[2] + values_[3] * values_[3]);
  }

  ALGTEM const T quaternion<T>::length() const{
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] +
        values_[2] * values_[2] + values_[3] * values_[3]);
  }

  template class quaternion<double>;
  template class quaternion<float>;
  template class quaternion<int>;
}
