
#include "algebraica/vec4.h"

namespace Algebraica {
  ALGTEM T& vec4<T>::operator[](const unsigned int index){
    assert(index >= 0 && index < 4);
    return values_[index];
  }
  ALGTEM const T& vec4<T>::operator[](const unsigned int index) const{
    assert(index >= 0 && index < 4);
    return values_[index];
  }

  ALGTEM vec4<T>& vec4<T>::operator()(const T x, const T y, const T z, const T w){
    values_[0] = x;
    values_[1] = y;
    values_[2] = z;
    values_[3] = w;
    return *this;
  }

  ALGTEM vec4<T> vec4<T>::operator+(const vec4<T> &v){
    return vec4<T>(values_[0] + v[0], values_[1] + v[1], values_[2] + v[2], values_[3] + v[3]);
  }

  ALGTEM const vec4<T> vec4<T>::operator+(const vec4<T> &v) const{
    return vec4<T>(values_[0] + v[0], values_[1] + v[1], values_[2] + v[2], values_[3] + v[3]);
  }

  ALGTEM vec4<T> vec4<T>::operator+(const T &s){
    return vec4<T>(values_[0] + s, values_[1] + s, values_[2] + s, values_[3] + s);
  }

  ALGTEM const vec4<T> vec4<T>::operator+(const T &s) const{
    return vec4<T>(values_[0] + s, values_[1] + s, values_[2] + s, values_[3] + s);
  }

  ALGTEM vec4<T> vec4<T>::operator-(const vec4<T> &v){
    return vec4<T>(values_[0] - v[0], values_[1] - v[1], values_[2] - v[2], values_[3] - v[3]);
  }

  ALGTEM const vec4<T> vec4<T>::operator-(const vec4<T> &v) const{
    return vec4<T>(values_[0] - v[0], values_[1] - v[1], values_[2] - v[2], values_[3] - v[3]);
  }

  ALGTEM vec4<T> vec4<T>::operator-(const T &s){
    return vec4<T>(values_[0] - s, values_[1] - s, values_[2] - s, values_[3] - s);
  }

  ALGTEM const vec4<T> vec4<T>::operator-(const T &s) const{
    return vec4<T>(values_[0] - s, values_[1] - s, values_[2] - s, values_[3] - s);
  }

  ALGTEM vec4<T> vec4<T>::operator*(const vec4<T> &v){
    return vec4<T>(values_[0] * v[0], values_[1] * v[1], values_[2] * v[2], values_[3] * v[3]);
  }

  ALGTEM const vec4<T> vec4<T>::operator*(const vec4<T> &v) const{
    return vec4<T>(values_[0] * v[0], values_[1] * v[1], values_[2] * v[2], values_[3] * v[3]);
  }

  ALGTEM vec4<T> vec4<T>::operator*(const vec3<T> &v){
    return vec4<T>(values_[0] * v[0], values_[1] * v[1], values_[2] * v[2], values_[3]);
  }

  ALGTEM vec4<T> vec4<T>::operator*(const T &s){
    return vec4<T>(values_[0] * s, values_[1] * s, values_[2] * s, values_[3] * s);
  }

  ALGTEM const vec4<T> vec4<T>::operator*(const T &s) const{
    return vec4<T>(values_[0] * s, values_[1] * s, values_[2] * s, values_[3] * s);
  }

  ALGTEM vec4<T> vec4<T>::operator/(const vec4<T> &v){
    return vec4<T>(values_[0] / v[0], values_[1] / v[1], values_[2] / v[2], values_[3] / v[3]);
  }

  ALGTEM const vec4<T> vec4<T>::operator/(const vec4<T> &v) const{
    return vec4<T>(values_[0] / v[0], values_[1] / v[1], values_[2] / v[2], values_[3] / v[3]);
  }

  ALGTEM vec4<T> vec4<T>::operator/(const T &s){
    return vec4<T>(values_[0] / s, values_[1] / s, values_[2] / s, values_[3] / s);
  }

  ALGTEM const vec4<T> vec4<T>::operator/(const T &s) const{
    return vec4<T>(values_[0] / s, values_[1] / s, values_[2] / s, values_[3] / s);
  }

  ALGTEM vec4<T>& vec4<T>::operator+=(const vec4<T> &v){
    values_[0] += v[0];
    values_[1] += v[1];
    values_[2] += v[2];
    values_[3] += v[3];
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator+=(const T &s){
    values_[0] += s;
    values_[1] += s;
    values_[2] += s;
    values_[3] += s;
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator-=(const vec4<T> &v){
    values_[0] -= v[0];
    values_[1] -= v[1];
    values_[2] -= v[2];
    values_[3] -= v[3];
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator-=(const T &s){
    values_[0] -= s;
    values_[1] -= s;
    values_[2] -= s;
    values_[3] -= s;
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator*=(const vec4<T> &v){
    values_[0] *= v[0];
    values_[1] *= v[1];
    values_[2] *= v[2];
    values_[3] *= v[3];
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator*=(const T &s){
    values_[0] *= s;
    values_[1] *= s;
    values_[2] *= s;
    values_[3] *= s;
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator/=(const vec4<T> &v){
    values_[0] /= v[0];
    values_[1] /= v[1];
    values_[2] /= v[2];
    values_[3] /= v[3];
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator/=(const T &s){
    values_[0] /= s;
    values_[1] /= s;
    values_[2] /= s;
    values_[3] /= s;
    return *this;
  }

  ALGTEM vec4<T>& vec4<T>::operator++(){
    ++values_[0];
    ++values_[1];
    ++values_[2];
    ++values_[3];
    return *this;
  }

  ALGTEM vec4<T> vec4<T>::operator++(int){
    vec4<T> temp(*this);
    ++*this;
    return temp;
  }

  ALGTEM vec4<T>& vec4<T>::operator--(){
    --values_[0];
    --values_[1];
    --values_[2];
    --values_[3];
    return *this;
  }

  ALGTEM vec4<T> vec4<T>::operator--(int){
    vec4<T> temp(*this);
    --*this;
    return temp;
  }

  ALGTEM bool vec4<T>::operator==(const vec4<T> &v){
    return values_[0] == v[0] && values_[1] == v[1] && values_[2] == v[2] && values_[3] == v[3];
  }

  ALGTEM const bool vec4<T>::operator==(const vec4<T> &v) const{
    return values_[0] == v[0] && values_[1] == v[1] && values_[2] == v[2] && values_[3] == v[3];
  }

  ALGTEM bool vec4<T>::operator!=(const vec4<T> &v){
    return !(values_[0] == v[0] && values_[1] == v[1] && values_[2] == v[2] && values_[3] == v[3]);
  }

  ALGTEM const bool vec4<T>::operator!=(const vec4<T> &v) const{
    return !(values_[0] == v[0] && values_[1] == v[1] && values_[2] == v[2] && values_[3] == v[3]);
  }

  ALGTEM bool vec4<T>::operator>(const vec4<T> &v){
    return length() > v.length();
  }

  ALGTEM const bool vec4<T>::operator>(const vec4<T> &v) const{
    return length() > v.length();
  }

  ALGTEM bool vec4<T>::operator<(const vec4<T> &v){
    return length() < v.length();
  }

  ALGTEM const bool vec4<T>::operator<(const vec4<T> &v) const{
    return length() < v.length();
  }

  ALGTEM bool vec4<T>::operator>=(const vec4<T> &v){
    return length() >= v.length();
  }

  ALGTEM const bool vec4<T>::operator>=(const vec4<T> &v) const{
    return length() >= v.length();
  }

  ALGTEM bool vec4<T>::operator<=(const vec4<T> &v){
    return length() <= v.length();
  }

  ALGTEM const bool vec4<T>::operator<=(const vec4<T> &v) const{
    return length() <= v.length();
  }

  ALGTEM const T* vec4<T>::data() const{
    return &values_[0];
  }

  ALGTEM T* vec4<T>::data(){
    return &values_[0];
  }

  ALGTEM T vec4<T>::length(){
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] +
        values_[2] * values_[2] + values_[3] * values_[3]);
  }

  ALGTEM const T vec4<T>::length() const{
    return std::sqrt(values_[0] * values_[0] + values_[1] * values_[1] +
        values_[2] * values_[2] + values_[3] * values_[3]);
  }

  ALGTEM vec4<T>& vec4<T>::normalize(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    values_[0] *= d;
    values_[1] *= d;
    values_[2] *= d;
    values_[3] *= d;
    return *this;
  }

  ALGTEM vec4<T> vec4<T>::normalized(){
    T d(length());
    d = (d > 0)? 1/d : 0;
    return vec4<T>(values_[0] * d, values_[1] * d, values_[2] * d, values_[3] * d);
  }

  ALGTEM const vec4<T> vec4<T>::normalized() const{
    T d(length());
    d = (d > 0)? 1/d : 0;
    return vec4<T>(values_[0] * d, values_[1] * d, values_[2] * d, values_[3] * d);
  }

  ALGTEM T vec4<T>::add(){
    return T(values_[0] + values_[1] + values_[2] + values_[3]);
  }

  ALGTEM const T vec4<T>::add() const{
    return T(values_[0] + values_[1] + values_[2] + values_[3]);
  }

  template class vec4<double>;
  template class vec4<float>;
  template class vec4<int>;
}
