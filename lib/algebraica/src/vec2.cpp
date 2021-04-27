
#include "algebraica/vec2.h"

namespace Algebraica {
  ALGTEM T& vec2<T>::operator[](const unsigned int index){
    return values_[index];
  }

  ALGTEM const T& vec2<T>::operator[](const unsigned int index) const{
    return values_[index];
  }

  ALGTEM vec2<T> vec2<T>::operator+(const vec2<T> &v){
    return vec2<T>(values_[0] + v[0], values_[1] + v[1]);
  }

  ALGTEM const vec2<T> vec2<T>::operator+(const vec2<T> &v) const{
    return vec2<T>(values_[0] + v[0], values_[1] + v[1]);
  }

  ALGTEM vec2<T> vec2<T>::operator-(vec2<T> &v){
    return vec2<T>(values_[0] - v[0], values_[1] - v[1]);
  }

  ALGTEM const vec2<T> vec2<T>::operator-(vec2<T> &v) const{
    return vec2<T>(values_[0] - v[0], values_[1] - v[1]);
  }

  ALGTEM vec2<T> vec2<T>::operator*(const vec2<T> &v){
    return vec2<T>(values_[0] * v[0], values_[1] * v[1]);
  }

  ALGTEM const vec2<T> vec2<T>::operator*(const vec2<T> &v) const{
    return vec2<T>(values_[0] * v[0], values_[1] * v[1]);
  }

  ALGTEM vec2<T> vec2<T>::operator/(const vec2<T> &v){
    return vec2<T>(values_[0] / v[0], values_[1] / v[1]);
  }

  ALGTEM const vec2<T> vec2<T>::operator/(const vec2<T> &v) const{
    return vec2<T>(values_[0] / v[0], values_[1] / v[1]);
  }

  ALGTEM vec2<T>& vec2<T>::operator+=(const vec2<T> &v){
    values_[0] += v[0];
    values_[1] += v[1];
    return *this;
  }

  ALGTEM vec2<T>& vec2<T>::operator-=(const vec2<T> &v){
    values_[0] -= v[0];
    values_[1] -= v[1];
    return *this;
  }

  ALGTEM vec2<T>& vec2<T>::operator*=(const vec2<T> &v){
    values_[0] *= v[0];
    values_[1] *= v[1];
    return *this;
  }

  ALGTEM vec2<T>& vec2<T>::operator/=(const vec2<T> &v){
    values_[0] /= v[0];
    values_[1] /= v[1];
    return *this;
  }

  ALGTEM vec2<T>& vec2<T>::operator++(){
    ++values_[0];
    ++values_[1];
    return *this;
  }

  ALGTEM vec2<T> vec2<T>::operator++(int){
    vec2<T> temp(*this);
    ++*this;
    return temp;
  }

  ALGTEM vec2<T>& vec2<T>::operator--(){
    --values_[0];
    --values_[1];
    return *this;
  }

  ALGTEM vec2<T> vec2<T>::operator--(int){
    vec2<T> temp(*this);
    --*this;
    return temp;
  }

  ALGTEM vec2<T>& vec2<T>::operator ()(const T x, const T y){
    values_[0] = x;
    values_[1] = y;
    return *this;
  }

  ALGTEM bool vec2<T>::operator==(const vec2<T> &v){
    return values_[0] == v[0] && values_[1] == v[1];
  }

  ALGTEM const bool vec2<T>::operator==(const vec2<T> &v) const{
    return values_[0] == v[0] && values_[1] == v[1];
  }

  ALGTEM bool vec2<T>::operator!=(const vec2<T> &v){
    return !(values_[0] == v[0] && values_[1] == v[1]);
  }

  ALGTEM const bool vec2<T>::operator!=(const vec2<T> &v) const{
    return !(values_[0] == v[0] && values_[1] == v[1]);
  }

  ALGTEM bool vec2<T>::operator>(const vec2<T> &v){
    return length() > v.length();
  }

  ALGTEM const bool vec2<T>::operator>(const vec2<T> &v) const{
    return length() > v.length();
  }

  ALGTEM bool vec2<T>::operator<(const vec2<T> &v){
    return length() < v.length();
  }

  ALGTEM const bool vec2<T>::operator<(const vec2<T> &v) const{
    return length() < v.length();
  }

  ALGTEM bool vec2<T>::operator>=(const vec2<T> &v){
    return length() >= v.length();
  }

  ALGTEM const bool vec2<T>::operator>=(const vec2<T> &v) const{
    return length() >= v.length();
  }

  ALGTEM bool vec2<T>::operator<=(const vec2<T> &v){
    return length() <= v.length();
  }

  ALGTEM const bool vec2<T>::operator<=(const vec2<T> &v) const{
    return length() <= v.length();
  }

  ALGTEM T* vec2<T>::data(){
    return &values_[0];
  }

  ALGTEM const T* vec2<T>::data() const{
    return &values_[0];
  }

  ALGTEM T vec2<T>::length(){
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1]);
  }

  ALGTEM const T vec2<T>::length() const{
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1]);
  }

  ALGTEM vec2<T>& vec2<T>::normalize(){
    T d(length());
    if(d > 0) d = 1 / d;
    values_[0] *= d;
    values_[1] *= d;
    return *this;
  }

  ALGTEM vec2<T> vec2<T>::normalized(){
    T d(length());
    if(d > 0) d = 1 / d;
    return vec2<T>(values_[0] * d, values_[1] * d);
  }

  ALGTEM const vec2<T> vec2<T>::normalized() const{
    T d(length());
    if(d > 0) d = 1 / d;
    return vec2<T>(values_[0] * d, values_[1] * d);
  }

  ALGTEM vec2<T>& vec2<T>::rotate(const T _angle){
    values_[0] = values_[0] * std::cos(_angle);
    values_[1] = values_[1] * std::sin(_angle);
    return *this;
  }

  template class vec2<double>;
  template class vec2<float>;
  template class vec2<int>;
}
