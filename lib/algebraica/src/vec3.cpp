#include "algebraica/vec3.h"
#include "algebraica/quaternion.h"

namespace Algebraica {
  ALGTEM T& vec3<T>::operator[](const unsigned int index){
    assert(index >= 0 && index < 3);
    return values_[index];
  }

  ALGTEM const T& vec3<T>::operator[](const unsigned int index) const{
    assert(index >= 0 && index < 3);
    return values_[index];
  }

  ALGTEM vec3<T> vec3<T>::operator+(const vec3<T> &v){
    return vec3<T>(values_[0] + v.x(), values_[1] + v.y(), values_[2] + v.z());
  }

  ALGTEM const vec3<T> vec3<T>::operator+(const vec3<T> &v) const{
    return vec3<T>(values_[0] + v.x(), values_[1] + v.y(), values_[2] + v.z());
  }

  ALGTEM vec3<T> vec3<T>::operator-(const vec3<T> &v){
    return vec3<T>(values_[0] - v.x(), values_[1] - v.y(), values_[2] - v.z());
  }

  ALGTEM const vec3<T> vec3<T>::operator-(const vec3<T> &v) const{
    return vec3<T>(values_[0] - v.x(), values_[1] - v.y(), values_[2] - v.z());
  }

  ALGTEM vec3<T> vec3<T>::operator*(const vec3<T> &v){
    return vec3<T>(values_[0] * v.x(), values_[1] * v.y(), values_[2] * v.z());
  }

  ALGTEM const vec3<T> vec3<T>::operator*(const vec3<T> &v) const{
    return vec3<T>(values_[0] * v.x(), values_[1] * v.y(), values_[2] * v.z());
  }

  ALGTEM vec3<T> vec3<T>::operator*(const T s){
    return vec3<T>(values_[0] * s, values_[1] * s, values_[2] * s);
  }

  ALGTEM const vec3<T> vec3<T>::operator*(const T s) const{
    return vec3<T>(values_[0] * s, values_[1] * s, values_[2] * s);
  }

  ALGTEM vec3<T> vec3<T>::operator/(const vec3<T> &v){
    return vec3<T>(values_[0] / v.x(), values_[1] / v.y(), values_[2] / v.z());
  }

  ALGTEM const vec3<T> vec3<T>::operator/(const vec3<T> &v) const{
    return vec3<T>(values_[0] / v.x(), values_[1] / v.y(), values_[2] / v.z());
  }

  ALGTEM vec3<T> vec3<T>::operator/(const T s){
    return vec3<T>(values_[0] / s, values_[1] / s, values_[2] / s);
  }

  ALGTEM const vec3<T> vec3<T>::operator/(const T s) const{
    return vec3<T>(values_[0] / s, values_[1] / s, values_[2] / s);
  }

  ALGTEM vec3<T>& vec3<T>::operator+=(const vec3<T> &v){
    values_[0] += v.x();
    values_[1] += v.y();
    values_[2] += v.z();
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator-=(const vec3<T> &v){
    values_[0] -= v.x();
    values_[1] -= v.y();
    values_[2] -= v.z();
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator*=(const vec3<T> &v){
    values_[0] *= v.x();
    values_[1] *= v.y();
    values_[2] *= v.z();
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator*=(const T s){
    values_[0] *= s;
    values_[1] *= s;
    values_[2] *= s;
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator/=(const vec3<T> &v){
    values_[0] /= v.x();
    values_[1] /= v.y();
    values_[2] /= v.z();
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator/=(const T s){
    values_[0] /= s;
    values_[1] /= s;
    values_[2] /= s;
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::operator++(){
    ++values_[0];
    ++values_[1];
    ++values_[2];
    return *this;
  }

  ALGTEM vec3<T> vec3<T>::operator++(int){
    vec3<T> temp(*this);
    ++*this;
    return temp;
  }

  ALGTEM vec3<T>& vec3<T>::operator--(){
    --values_[0];
    --values_[1];
    --values_[2];
    return *this;
  }

  ALGTEM vec3<T> vec3<T>::operator--(int){
    vec3<T> temp(*this);
    --*this;
    return temp;
  }

  ALGTEM vec3<T>& vec3<T>::operator ()(const T x, const T y, const T z){
    values_[0] = x;
    values_[1] = y;
    values_[2] = z;
    return *this;
  }

  ALGTEM bool vec3<T>::operator==(const vec3<T> &v){
    return values_[0] == v.x() && values_[1] == v.y() && values_[2] == v.z();
  }

  ALGTEM const bool vec3<T>::operator==(const vec3<T> &v) const{
    return values_[0] == v.x() && values_[1] == v.y() && values_[2] == v.z();
  }

  ALGTEM bool vec3<T>::operator!=(const vec3<T> &v){
    return !(values_[0] == v.x() && values_[1] == v.y() && values_[2] == v.z());
  }

  ALGTEM const bool vec3<T>::operator!=(const vec3<T> &v) const{
    return !(values_[0] == v.x() && values_[1] == v.y() && values_[2] == v.z());
  }

  ALGTEM bool vec3<T>::operator>(const vec3<T> &v){
    return length() > v.length();
  }

  ALGTEM const bool vec3<T>::operator>(const vec3<T> &v) const{
    return length() > v.length();
  }

  ALGTEM bool vec3<T>::operator<(const vec3<T> &v){
    return length() < v.length();
  }

  ALGTEM const bool vec3<T>::operator<(const vec3<T> &v) const{
    return length() < v.length();
  }

  ALGTEM bool vec3<T>::operator>=(const vec3<T> &v){
    return length() >= v.length();
  }

  ALGTEM const bool vec3<T>::operator>=(const vec3<T> &v) const{
    return length() >= v.length();
  }

  ALGTEM bool vec3<T>::operator<=(const vec3<T> &v){
    return length() <= v.length();
  }

  ALGTEM const bool vec3<T>::operator<=(const vec3<T> &v) const{
    return length() <= v.length();
  }

  ALGTEM T* vec3<T>::data(){
    return &values_[0];
  }

  ALGTEM const T* vec3<T>::data() const{
    return &values_[0];
  }

  ALGTEM T vec3<T>::length(){
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] + values_[2] * values_[2]);
  }

  ALGTEM const T vec3<T>::length() const{
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] + values_[2] * values_[2]);
  }

  ALGTEM T vec3<T>::distance(const vec3<T> &first, const vec3<T> &second){
    return vec3<T>(first - second).length();
  }

  ALGTEM vec3<T>& vec3<T>::normalize(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    values_[0] *= d;
    values_[1] *= d;
    values_[2] *= d;
    return *this;
  }

  ALGTEM vec3<T> vec3<T>::normalize(vec3<T> v){
    T d{v.length()};
    d = (d > 0)? 1/d : 0;
    return vec3<T>(d * v.x(), d * v.y(), d * v.z());
  }

  ALGTEM vec3<T> vec3<T>::normalized(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    return vec3<T>(values_[0] * d, values_[1] * d, values_[2] * d);
  }

  ALGTEM const vec3<T> vec3<T>::normalized() const{
    T d(length());
    d = (d > 0)? 1/d : 0;
    return vec3<T>(values_[0] * d, values_[1] * d, values_[2] * d);
  }

  ALGTEM vec3<T> vec3<T>::cross(const vec3<T> &u, const vec3<T> &v){
    return vec3<T>(u.y() * v.z() - v.y() * u.z(),
                   u.z() * v.x() - v.z() * u.x(),
                   u.x() * v.y() - v.x() * u.y());
  }

  ALGTEM T vec3<T>::dot(vec3<T> &u, const vec3<T> &v){
    vec3<T> tmp(u * v);
    return tmp.x() + tmp.y() + tmp.z();
  }

  ALGTEM vec3<T>& vec3<T>::rotateX(const T _angle) {
    values_[1] = values_[1] * std::cos(_angle) - values_[2] * std::sin(_angle);
    values_[2] = values_[1] * std::sin(_angle) + values_[2] * std::cos(_angle);
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::rotateY(const T _angle) {
    values_[0] = values_[2] * std::sin(_angle) + values_[0] * std::cos(_angle);
    values_[2] = values_[2] * std::cos(_angle) - values_[0] * std::sin(_angle);
    return *this;
  }

  ALGTEM vec3<T>& vec3<T>::rotateZ(const T _angle) {
    values_[0] = values_[0] * std::cos(_angle) - values_[1] * std::sin(_angle);
    values_[1] = values_[0] * std::sin(_angle) + values_[1] * std::cos(_angle);
    return *this;
  }

  ALGTEM vec3<T> &vec3<T>::rotate(const quaternion<T> quat){
    const vec3<T> Q{quat.to_3D()};
    const vec3<T> A{cross(*this, Q) + (*this) * quat[3]};
    *this += cross(A, Q) * T(2);
    return *this;
  }

  template class vec3<double>;
  template class vec3<float>;
  template class vec3<int>;
}
