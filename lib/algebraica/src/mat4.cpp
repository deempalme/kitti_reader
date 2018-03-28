
#include "algebraica/mat4.h"

namespace Algebraica {
  ALGTEM mat4<T>& mat4<T>::to_identity(){
    m_[0] = vec4<T>(1, 0, 0, 0);
    m_[1] = vec4<T>(0, 1, 0, 0);
    m_[2] = vec4<T>(0, 0, 1, 0);
    m_[3] = vec4<T>(0, 0, 0, 1);
    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::to_zero(){
    m_[0] = vec4<T>(0, 0, 0, 0);
    m_[1] = vec4<T>(0, 0, 0, 0);
    m_[2] = vec4<T>(0, 0, 0, 0);
    m_[3] = vec4<T>(0, 0, 0, 0);
    return *this;
  }

  ALGTEM T mat4<T>::determinant(){
    T SubFactor00 = m_[2][2] * m_[3][3] - m_[3][2] * m_[2][3];
    T SubFactor01 = m_[2][1] * m_[3][3] - m_[3][1] * m_[2][3];
    T SubFactor02 = m_[2][1] * m_[3][2] - m_[3][1] * m_[2][2];
    T SubFactor03 = m_[2][0] * m_[3][3] - m_[3][0] * m_[2][3];
    T SubFactor04 = m_[2][0] * m_[3][2] - m_[3][0] * m_[2][2];
    T SubFactor05 = m_[2][0] * m_[3][1] - m_[3][0] * m_[2][1];

    T DetCof[4]{(m_[1][1] * SubFactor00 - m_[1][2] * SubFactor01 + m_[1][3] * SubFactor02),
          -(m_[1][0] * SubFactor00 - m_[1][2] * SubFactor03 + m_[1][3] * SubFactor04),
          +(m_[1][0] * SubFactor01 - m_[1][1] * SubFactor03 + m_[1][3] * SubFactor05),
          -(m_[1][0] * SubFactor02 - m_[1][1] * SubFactor04 + m_[1][2] * SubFactor05)};

    return m_[0][0] * DetCof[0] + m_[0][1] * DetCof[1] +
        m_[0][2] * DetCof[2] + m_[0][3] * DetCof[3];
  }

  ALGTEM const T mat4<T>::determinant() const{
    T SubFactor00 = m_[2][2] * m_[3][3] - m_[3][2] * m_[2][3];
    T SubFactor01 = m_[2][1] * m_[3][3] - m_[3][1] * m_[2][3];
    T SubFactor02 = m_[2][1] * m_[3][2] - m_[3][1] * m_[2][2];
    T SubFactor03 = m_[2][0] * m_[3][3] - m_[3][0] * m_[2][3];
    T SubFactor04 = m_[2][0] * m_[3][2] - m_[3][0] * m_[2][2];
    T SubFactor05 = m_[2][0] * m_[3][1] - m_[3][0] * m_[2][1];

    T DetCof[4]{(m_[1][1] * SubFactor00 - m_[1][2] * SubFactor01 + m_[1][3] * SubFactor02),
          -(m_[1][0] * SubFactor00 - m_[1][2] * SubFactor03 + m_[1][3] * SubFactor04),
          +(m_[1][0] * SubFactor01 - m_[1][1] * SubFactor03 + m_[1][3] * SubFactor05),
          -(m_[1][0] * SubFactor02 - m_[1][1] * SubFactor04 + m_[1][2] * SubFactor05)};

    return m_[0][0] * DetCof[0] + m_[0][1] * DetCof[1] +
        m_[0][2] * DetCof[2] + m_[0][3] * DetCof[3];
  }

  ALGTEM mat4<T>& mat4<T>::rotate(const T angle, const vec3<T> &v){
    T const c{static_cast<T>(std::cos(angle))}, t{static_cast<T>(1) - c};
    T const s{static_cast<T>(std::sin(angle))};

    T x{v[0]}, y{v[1]}, z{v[2]}, lenght{static_cast<T>(std::sqrt(x * x + y * y + z * z))};

    if(lenght < 0.001)
      return *this;

    x /= lenght;
    y /= lenght;
    z /= lenght;

    T temp[3]{t * x, t * y, t * z};

    T Rotate[9];
    Rotate[0] = c + temp[0] * x;
    Rotate[1] = temp[0] * y + s * z;
    Rotate[2] = temp[0] * z - s * y;

    Rotate[3] = temp[1] * x - s * z;
    Rotate[4] = c + temp[1] * y;
    Rotate[5] = temp[1] * z + s * x;

    Rotate[6] = temp[2] * x + s * y;
    Rotate[7] = temp[2] * y - s * x;
    Rotate[8] = c + temp[2] * z;

    vec4<T> m0(m_[0]), m1(m_[1]), m2(m_[2]);

    m_[0] = m0 * Rotate[0] + m1 * Rotate[1] + m2 * Rotate[2];
    m_[1] = m0 * Rotate[3] + m1 * Rotate[4] + m2 * Rotate[5];
    m_[2] = m0 * Rotate[6] + m1 * Rotate[7] + m2 * Rotate[8];

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::rotate(const T pitch, const T yaw, const T roll){
    const T cosA{static_cast<T>(std::cos(pitch))}, sinA{static_cast<T>(std::sin(pitch))};
    const T cosB{static_cast<T>(std::cos(yaw))},   sinB{static_cast<T>(std::sin(yaw))};
    const T cosC{static_cast<T>(std::cos(roll))},  sinC{static_cast<T>(std::sin(roll))};

    mat4<T> r_m(cosC * cosB - sinC * sinA * sinB,-sinC * cosA, cosC * sinB + sinC * sinA * cosB, 0,
                sinC * cosB + cosC * sinA * sinB, cosC * cosA, sinC * sinB - cosC * sinA * cosB, 0,
                                    -cosA * sinB,        sinA,                      cosA * cosB, 0,
                                               0,           0,                                0, 1);

    return *this = *this * r_m;
  }

  ALGTEM mat4<T>& mat4<T>::from_euler(const T pitch, const T yaw, const T roll){
    const T cosA{static_cast<T>(std::cos(pitch))}, sinA{static_cast<T>(std::sin(pitch))};
    const T cosB{static_cast<T>(std::cos(yaw))},   sinB{static_cast<T>(std::sin(yaw))};
    const T cosC{static_cast<T>(std::cos(roll))},  sinC{static_cast<T>(std::sin(roll))};

    mat4<T> r_m(cosC * cosB - sinC * sinA * sinB,-sinC * cosA, cosC * sinB + sinC * sinA * cosB, 0,
                sinC * cosB + cosC * sinA * sinB, cosC * cosA, sinC * sinB - cosC * sinA * cosB, 0,
                                    -cosA * sinB,        sinA,                      cosA * cosB, 0,
                                               0,           0,                                0, 1);

    return *this = r_m;
  }

  ALGTEM mat4<T>& mat4<T>::rotate(const T x, const T y, const T z, const T w){
    mat4<T> r_m;

    const T one = static_cast<T>(1);
    const T two = static_cast<T>(2);

    const T xx = x * x;
    const T xy = x * y;
    const T xz = x * z;
    const T xw = x * w;

    const T yy = y * y;
    const T yz = y * z;
    const T yw = y * w;

    const T zz = z * z;
    const T zw = z * w;

    r_m[0][0] = one - two * (yy + zz);
    r_m[0][1] =       two * (xy - zw);
    r_m[0][2] =       two * (xz + yw);

    r_m[1][0] =       two * (xy + zw);
    r_m[1][1] = one - two * (xx + zz);
    r_m[1][2] =       two * (yz - xw);

    r_m[2][0] =       two * (xz - yw);
    r_m[2][1] =       two * (yz + xw);
    r_m[2][2] = one - two * (xx + yy);

    r_m[0][3] = r_m[1][3] = r_m[2][3] = r_m[3][0] = r_m[3][1] = r_m[3][2] = static_cast<T>(0);
    r_m[3][3] = one;

    return *this = *this * r_m;
  }

  ALGTEM mat4<T>& mat4<T>::rotate(const quaternion<T> quat){
    mat4<T> r_m;

    const T one = static_cast<T>(1);
    const T two = static_cast<T>(2);

    const T xx = quat.x() * quat.x();
    const T xy = quat.x() * quat.y();
    const T xz = quat.x() * quat.z();
    const T xw = quat.x() * quat.w();

    const T yy = quat.y() * quat.y();
    const T yz = quat.y() * quat.z();
    const T yw = quat.y() * quat.w();

    const T zz = quat.z() * quat.z();
    const T zw = quat.z() * quat.w();

    r_m[0][0] = one - two * (yy + zz);
    r_m[0][1] =       two * (xy - zw);
    r_m[0][2] =       two * (xz + yw);

    r_m[1][0] =       two * (xy + zw);
    r_m[1][1] = one - two * (xx + zz);
    r_m[1][2] =       two * (yz - xw);

    r_m[2][0] =       two * (xz - yw);
    r_m[2][1] =       two * (yz + xw);
    r_m[2][2] = one - two * (xx + yy);

    r_m[0][3] = r_m[1][3] = r_m[2][3] = r_m[3][0] = r_m[3][1] = r_m[3][2] = static_cast<T>(0);
    r_m[3][3] = one;

    return *this = *this * r_m;
  }

  ALGTEM mat4<T>& mat4<T>::from_quaternion(const T x, const T y, const T z, const T w){
    const T one = static_cast<T>(1);
    const T two = static_cast<T>(2);

    const T xx = x * x;
    const T xy = x * y;
    const T xz = x * z;
    const T xw = x * w;

    const T yy = y * y;
    const T yz = y * z;
    const T yw = y * w;

    const T zz = z * z;
    const T zw = z * w;

    m_[0][0] = one - two * (yy + zz);
    m_[0][1] =       two * (xy - zw);
    m_[0][2] =       two * (xz + yw);

    m_[1][0] =       two * (xy + zw);
    m_[1][1] = one - two * (xx + zz);
    m_[1][2] =       two * (yz - xw);

    m_[2][0] =       two * (xz - yw);
    m_[2][1] =       two * (yz + xw);
    m_[2][2] = one - two * (xx + yy);

    m_[0][3] = m_[1][3] = m_[2][3] = m_[3][0] = m_[3][1] = m_[3][2] = static_cast<T>(0);
    m_[3][3] = one;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::from_quaternion(const quaternion<T> quat){
    const T one = static_cast<T>(1);
    const T two = static_cast<T>(2);

    const T xx = quat.x() * quat.x();
    const T xy = quat.x() * quat.y();
    const T xz = quat.x() * quat.z();
    const T xw = quat.x() * quat.w();

    const T yy = quat.y() * quat.y();
    const T yz = quat.y() * quat.z();
    const T yw = quat.y() * quat.w();

    const T zz = quat.z() * quat.z();
    const T zw = quat.z() * quat.w();

    m_[0][0] = one - two * (yy + zz);
    m_[0][1] =       two * (xy - zw);
    m_[0][2] =       two * (xz + yw);

    m_[1][0] =       two * (xy + zw);
    m_[1][1] = one - two * (xx + zz);
    m_[1][2] =       two * (yz - xw);

    m_[2][0] =       two * (xz - yw);
    m_[2][1] =       two * (yz + xw);
    m_[2][2] = one - two * (xx + yy);

    m_[0][3] = m_[1][3] = m_[2][3] = m_[3][0] = m_[3][1] = m_[3][2] = static_cast<T>(0);
    m_[3][3] = one;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::normalize_and_rotate(const T non_normalized_x,
                                                const T non_normalized_y,
                                                const T non_normalized_z,
                                                const T non_normalized_w){
    mat4<T> r_m;
    const T sqw = non_normalized_w * non_normalized_w;
    const T sqx = non_normalized_x * non_normalized_x;
    const T sqy = non_normalized_y * non_normalized_y;
    const T sqz = non_normalized_z * non_normalized_z;

    const T two = static_cast<T>(2.0);

    // invs (inverse square length) is only required if quaternion is not already normalised
    const T invs = static_cast<T>(1.0) / (sqx + sqy + sqz + sqw);
    r_m[0][0] = ( sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    r_m[1][1] = (-sqx + sqy - sqz + sqw) * invs;
    r_m[2][2] = (-sqx - sqy + sqz + sqw) * invs;

    T tmp1 = non_normalized_x * non_normalized_y;
    T tmp2 = non_normalized_z * non_normalized_w;
    r_m[1][0] = two * (tmp1 + tmp2) * invs;
    r_m[0][1] = two * (tmp1 - tmp2) * invs;

    tmp1 = non_normalized_x * non_normalized_z;
    tmp2 = non_normalized_y * non_normalized_w;
    r_m[2][0] = two * (tmp1 - tmp2) * invs;
    r_m[0][2] = two * (tmp1 + tmp2) * invs;
    tmp1 = non_normalized_y * non_normalized_z;
    tmp2 = non_normalized_x * non_normalized_w;
    r_m[2][1] = two * (tmp1 + tmp2) * invs;
    r_m[1][2] = two * (tmp1 - tmp2) * invs;

    return *this = *this * r_m;
  }

  ALGTEM mat4<T>& mat4<T>::normalize_and_rotate(const quaternion<T> non_normalized_quaternion){
    mat4<T> r_m;
    const T sqw = non_normalized_quaternion.w() * non_normalized_quaternion.w();
    const T sqx = non_normalized_quaternion.x() * non_normalized_quaternion.x();
    const T sqy = non_normalized_quaternion.y() * non_normalized_quaternion.y();
    const T sqz = non_normalized_quaternion.z() * non_normalized_quaternion.z();

    const T two = static_cast<T>(2.0);

    // invs (inverse square length) is only required if quaternion is not already normalised
    const T invs = static_cast<T>(1.0) / (sqx + sqy + sqz + sqw);
    r_m[0][0] = ( sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    r_m[1][1] = (-sqx + sqy - sqz + sqw) * invs;
    r_m[2][2] = (-sqx - sqy + sqz + sqw) * invs;

    T tmp1 = non_normalized_quaternion.x() * non_normalized_quaternion.y();
    T tmp2 = non_normalized_quaternion.z() * non_normalized_quaternion.w();
    r_m[1][0] = two * (tmp1 + tmp2) * invs;
    r_m[0][1] = two * (tmp1 - tmp2) * invs;

    tmp1 = non_normalized_quaternion.x() * non_normalized_quaternion.z();
    tmp2 = non_normalized_quaternion.y() * non_normalized_quaternion.w();
    r_m[2][0] = two * (tmp1 - tmp2) * invs;
    r_m[0][2] = two * (tmp1 + tmp2) * invs;
    tmp1 = non_normalized_quaternion.y() * non_normalized_quaternion.z();
    tmp2 = non_normalized_quaternion.x() * non_normalized_quaternion.w();
    r_m[2][1] = two * (tmp1 + tmp2) * invs;
    r_m[1][2] = two * (tmp1 - tmp2) * invs;

    return *this = *this * r_m;
  }

  ALGTEM mat4<T>& mat4<T>::from_non_normalized_quaternion(const T non_normalized_x,
                                                          const T non_normalized_y,
                                                          const T non_normalized_z,
                                                          const T non_normalized_w){
    const T sqw = non_normalized_w * non_normalized_w;
    const T sqx = non_normalized_x * non_normalized_x;
    const T sqy = non_normalized_y * non_normalized_y;
    const T sqz = non_normalized_z * non_normalized_z;

    const T two = static_cast<T>(2.0);

    // invs (inverse square length) is only required if quaternion is not already normalised
    const T invs = static_cast<T>(1.0) / (sqx + sqy + sqz + sqw);
    m_[0][0] = ( sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    m_[1][1] = (-sqx + sqy - sqz + sqw) * invs;
    m_[2][2] = (-sqx - sqy + sqz + sqw) * invs;

    T tmp1 = non_normalized_x * non_normalized_y;
    T tmp2 = non_normalized_z * non_normalized_w;
    m_[1][0] = two * (tmp1 + tmp2) * invs;
    m_[0][1] = two * (tmp1 - tmp2) * invs;

    tmp1 = non_normalized_x * non_normalized_z;
    tmp2 = non_normalized_y * non_normalized_w;
    m_[2][0] = two * (tmp1 - tmp2) * invs;
    m_[0][2] = two * (tmp1 + tmp2) * invs;
    tmp1 = non_normalized_y * non_normalized_z;
    tmp2 = non_normalized_x * non_normalized_w;
    m_[2][1] = two * (tmp1 + tmp2) * invs;
    m_[1][2] = two * (tmp1 - tmp2) * invs;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::from_non_normalized_quaternion(const quaternion<T>
                                                          non_normalized_quaternion){
    const T sqw = non_normalized_quaternion.w() * non_normalized_quaternion.w();
    const T sqx = non_normalized_quaternion.x() * non_normalized_quaternion.x();
    const T sqy = non_normalized_quaternion.y() * non_normalized_quaternion.y();
    const T sqz = non_normalized_quaternion.z() * non_normalized_quaternion.z();

    const T two = static_cast<T>(2.0);

    // invs (inverse square length) is only required if quaternion is not already normalised
    const T invs = static_cast<T>(1.0) / (sqx + sqy + sqz + sqw);
    m_[0][0] = ( sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    m_[1][1] = (-sqx + sqy - sqz + sqw) * invs;
    m_[2][2] = (-sqx - sqy + sqz + sqw) * invs;

    T tmp1 = non_normalized_quaternion.x() * non_normalized_quaternion.y();
    T tmp2 = non_normalized_quaternion.z() * non_normalized_quaternion.w();
    m_[1][0] = two * (tmp1 + tmp2) * invs;
    m_[0][1] = two * (tmp1 - tmp2) * invs;

    tmp1 = non_normalized_quaternion.x() * non_normalized_quaternion.z();
    tmp2 = non_normalized_quaternion.y() * non_normalized_quaternion.w();
    m_[2][0] = two * (tmp1 - tmp2) * invs;
    m_[0][2] = two * (tmp1 + tmp2) * invs;
    tmp1 = non_normalized_quaternion.y() * non_normalized_quaternion.z();
    tmp2 = non_normalized_quaternion.x() * non_normalized_quaternion.w();
    m_[2][1] = two * (tmp1 + tmp2) * invs;
    m_[1][2] = two * (tmp1 - tmp2) * invs;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::rotate_x(const T angle){
    T s{static_cast<T>(std::sin(angle))};
    T c{static_cast<T>(std::cos(angle))};

    vec4<T> m1(m_[1]), m2(m_[2]);

    m_[1] = m1 * c + m2 * s;
    m_[2] = m2 * c - m1 * s;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::rotate_y(const T angle){
    T s{static_cast<T>(std::sin(angle))};
    T c{static_cast<T>(std::cos(angle))};

    vec4<T> m0(m_[0]), m2(m_[2]);

    m_[0] = m0 * c - m2 * s;
    m_[2] = m0 * s + m2 * c;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::rotate_z(const T angle){
    T s{static_cast<T>(std::sin(angle))};
    T c{static_cast<T>(std::cos(angle))};

    vec4<T> m0(m_[0]), m1(m_[1]);

    m_[0] = m0 * c + m1 * s;
    m_[1] = m1 * c - m0 * s;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::translate(const vec3<T> &v){
    return translate(v[0], v[1], v[2]);
  }

  ALGTEM mat4<T>& mat4<T>::translate(const T x, const T y, const T z){
    m_[3] = m_[0] * x + m_[1] * y + m_[2] * z + m_[3];
    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::scale(const T s){
    return scale(s, s, s);
  }

  ALGTEM mat4<T>& mat4<T>::scale(const vec3<T> &v){
    return scale(v[0], v[1], v[2]);
  }

  ALGTEM mat4<T>& mat4<T>::scale(const T x, const T y, const T z){
    m_[0] *= x;
    m_[1] *= y;
    m_[2] *= z;

    return *this;
  }

  ALGTEM mat3<T> mat4<T>::normal_matrix(){
    T Determinant =
        + m_[0][0] * (m_[1][1] * m_[2][2] - m_[1][2] * m_[2][1])
        - m_[0][1] * (m_[1][0] * m_[2][2] - m_[1][2] * m_[2][0])
        + m_[0][2] * (m_[1][0] * m_[2][1] - m_[1][1] * m_[2][0]);

    mat3<T> Inverse;
    Inverse[0][0] =  (m_[1][1] * m_[2][2] - m_[2][1] * m_[1][2]);
    Inverse[0][1] = -(m_[1][0] * m_[2][2] - m_[2][0] * m_[1][2]);
    Inverse[0][2] =  (m_[1][0] * m_[2][1] - m_[2][0] * m_[1][1]);
    Inverse[1][0] = -(m_[0][1] * m_[2][2] - m_[2][1] * m_[0][2]);
    Inverse[1][1] =  (m_[0][0] * m_[2][2] - m_[2][0] * m_[0][2]);
    Inverse[1][2] = -(m_[0][0] * m_[2][1] - m_[2][0] * m_[0][1]);
    Inverse[2][0] =  (m_[0][1] * m_[1][2] - m_[1][1] * m_[0][2]);
    Inverse[2][1] = -(m_[0][0] * m_[1][2] - m_[1][0] * m_[0][2]);
    Inverse[2][2] =  (m_[0][0] * m_[1][1] - m_[1][0] * m_[0][1]);
    Inverse /= Determinant;

    return Inverse;
  }

  ALGTEM const mat3<T> mat4<T>::normal_matrix() const{
    T Determinant =
        + m_[0][0] * (m_[1][1] * m_[2][2] - m_[1][2] * m_[2][1])
        - m_[0][1] * (m_[1][0] * m_[2][2] - m_[1][2] * m_[2][0])
        + m_[0][2] * (m_[1][0] * m_[2][1] - m_[1][1] * m_[2][0]);

    mat3<T> Inverse;
    Inverse[0][0] =  (m_[1][1] * m_[2][2] - m_[2][1] * m_[1][2]);
    Inverse[0][1] = -(m_[1][0] * m_[2][2] - m_[2][0] * m_[1][2]);
    Inverse[0][2] =  (m_[1][0] * m_[2][1] - m_[2][0] * m_[1][1]);
    Inverse[1][0] = -(m_[0][1] * m_[2][2] - m_[2][1] * m_[0][2]);
    Inverse[1][1] =  (m_[0][0] * m_[2][2] - m_[2][0] * m_[0][2]);
    Inverse[1][2] = -(m_[0][0] * m_[2][1] - m_[2][0] * m_[0][1]);
    Inverse[2][0] =  (m_[0][1] * m_[1][2] - m_[1][1] * m_[0][2]);
    Inverse[2][1] = -(m_[0][0] * m_[1][2] - m_[1][0] * m_[0][2]);
    Inverse[2][2] =  (m_[0][0] * m_[1][1] - m_[1][0] * m_[0][1]);
    Inverse /= Determinant;

    return Inverse;
  }

  ALGTEM mat4<T>& mat4<T>::ortho(const T left, const T right, const T bottom, const T top,
                                 const T near_plane, const T far_plane){
    to_identity();
    m_[0][0] = static_cast<T>(2) / (right - left);
    m_[1][1] = static_cast<T>(2) / (top - bottom);
    m_[2][2] = - static_cast<T>(2) / (far_plane - near_plane);
    m_[3][0] = - (right + left) / (right - left);
    m_[3][1] = - (top + bottom) / (top - bottom);
    m_[3][2] = - (far_plane + near_plane) / (far_plane - near_plane);

    return *this;
  }

  ALGTEM mat4<T> mat4<T>::from_ortho(const T left, const T right, const T bottom, const T top,
                                      const T near_plane, const T far_plane){
    mat4<T> ortho;

    ortho[0][0] = static_cast<T>(2) / (right - left);
    ortho[1][1] = static_cast<T>(2) / (top - bottom);
    ortho[2][2] = - static_cast<T>(2) / (far_plane - near_plane);
    ortho[3][0] = - (right + left) / (right - left);
    ortho[3][1] = - (top + bottom) / (top - bottom);
    ortho[3][2] = - (far_plane + near_plane) / (far_plane - near_plane);

    return ortho;
  }

  ALGTEM mat4<T>& mat4<T>::perspective(const T fov, const T aspect, const T near, const T far){
    T const tanHalfFov = std::tan(fov / static_cast<T>(2));

    to_zero();

    m_[0][0] = static_cast<T>(1) / (aspect * tanHalfFov);
    m_[1][1] = static_cast<T>(1) / (tanHalfFov);
    m_[2][2] = - (far + near) / (far - near);
    m_[2][3] = - static_cast<T>(1);
    m_[3][2] = - (static_cast<T>(2) * far * near) / (far - near);

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::look_at(vec3<T> eye, vec3<T> center, vec3<T> up){
    vec3<T> f(vec3<T>::normalize(center - eye));
    vec3<T> s(vec3<T>::normalize(vec3<T>::cross(f, up)));
    vec3<T> u(vec3<T>::cross(s, f));

    to_identity();

    m_[0][0] = s[0];
    m_[1][0] = s[1];
    m_[2][0] = s[2];
    m_[0][1] = u[0];
    m_[1][1] = u[1];
    m_[2][1] = u[2];
    m_[0][2] =-f[0];
    m_[1][2] =-f[1];
    m_[2][2] =-f[2];
    m_[3][0] =-vec3<T>::dot(s, eye);
    m_[3][1] =-vec3<T>::dot(u, eye);
    m_[3][2] = vec3<T>::dot(f, eye);

    return *this;
  }

  ALGTEM mat4<T> mat4<T>::lookAt(vec3<T> eye, vec3<T> center, vec3<T> up){
    vec3<T> f(vec3<T>::normalize(center - eye));
    vec3<T> s(vec3<T>::normalize(vec3<T>::cross(f, up)));
    vec3<T> u(vec3<T>::cross(s, f));

    mat4<T> matrix;

    matrix[0][0] = s[0];
    matrix[1][0] = s[1];
    matrix[2][0] = s[2];
    matrix[0][1] = u[0];
    matrix[1][1] = u[1];
    matrix[2][1] = u[2];
    matrix[0][2] =-f[0];
    matrix[1][2] =-f[1];
    matrix[2][2] =-f[2];
    matrix[3][0] =-vec3<T>::dot(s, eye);
    matrix[3][1] =-vec3<T>::dot(u, eye);
    matrix[3][2] = vec3<T>::dot(f, eye);

    return matrix;
  }

  ALGTEM mat4<T> mat4<T>::stationary(){
    mat4<T> stationary_(*this);
    stationary_[3] = vec4<T>(0.0f, 0.0f, 0.0f, 1.0f);
    stationary_[0][3] = 0.0f;
    stationary_[1][3] = 0.0f;
    stationary_[2][3] = 0.0f;
    return stationary_;
  }

  ALGTEM const mat4<T> mat4<T>::stationary() const{
    mat4<T> stationary_(*this);
    stationary_[3] = vec4<T>(0.0f, 0.0f, 0.0f, 1.0f);
    stationary_[0][3] = 0.0f;
    stationary_[1][3] = 0.0f;
    stationary_[2][3] = 0.0f;
    return stationary_;
  }

  ALGTEM mat4<T> mat4<T>::only_translation(){
    mat4<T> translation_;
    translation_[3] = m_[3];
    return translation_;
  }

  ALGTEM const mat4<T> mat4<T>::only_translation() const{
    mat4<T> translation_;
    translation_[3] = m_[3];
    return translation_;
  }

  ALGTEM vec4<T>& mat4<T>::operator[](const int index){
    assert(index >= 0 && index < 4);
    return m_[index];
  }

  ALGTEM const vec4<T>& mat4<T>::operator[](const int index) const{
    assert(index >= 0 && index < 4);
    return m_[index];
  }

  ALGTEM mat4<T> mat4<T>::operator+(const T s){
    return mat4<T>(m_[0] + s, m_[1] + s, m_[2] + s, m_[3] + s);
  }

  ALGTEM const mat4<T> mat4<T>::operator+(const T s) const{
    return mat4<T>(m_[0] + s, m_[1] + s, m_[2] + s, m_[3] + s);
  }

  ALGTEM mat4<T> mat4<T>::operator+(const mat4<T> &m){
    return mat4<T>(m_[0] + m[0], m_[1] + m[1], m_[2] + m[2], m_[3] + m[3]);
  }

  ALGTEM const mat4<T> mat4<T>::operator+(const mat4<T> &m) const{
    return mat4<T>(m_[0] + m[0], m_[1] + m[1], m_[2] + m[2], m_[3] + m[3]);
  }

  ALGTEM mat4<T> mat4<T>::operator-(const T s){
    return mat4<T>(m_[0] - s, m_[1] - s, m_[2] - s, m_[3] - s);
  }

  ALGTEM const mat4<T> mat4<T>::operator-(const T s) const{
    return mat4<T>(m_[0] - s, m_[1] - s, m_[2] - s, m_[3] - s);
  }

  ALGTEM mat4<T> mat4<T>::operator-(const mat4<T> &m){
    return mat4<T>(m_[0] - m[0], m_[1] - m[1], m_[2] - m[2], m_[3] - m[3]);
  }

  ALGTEM const mat4<T> mat4<T>::operator-(const mat4<T> &m) const{
    return mat4<T>(m_[0] - m[0], m_[1] - m[1], m_[2] - m[2], m_[3] - m[3]);
  }

  ALGTEM mat4<T> mat4<T>::operator*(const T s){
    return mat4<T>(m_[0] * s, m_[1] * s, m_[2] * s, m_[3] * s);
  }

  ALGTEM const mat4<T> mat4<T>::operator*(const T s) const{
    return mat4<T>(m_[0] * s, m_[1] * s, m_[2] * s, m_[3] * s);
  }

  ALGTEM mat4<T> mat4<T>::operator*(const mat4<T> &m){
    mat4<T> out;
    out[0] = m_[0] * m[0][0] + m_[1] * m[0][1] + m_[2] * m[0][2] + m_[3] * m[0][3];
    out[1] = m_[0] * m[1][0] + m_[1] * m[1][1] + m_[2] * m[1][2] + m_[3] * m[1][3];
    out[2] = m_[0] * m[2][0] + m_[1] * m[2][1] + m_[2] * m[2][2] + m_[3] * m[2][3];
    out[3] = m_[0] * m[3][0] + m_[1] * m[3][1] + m_[2] * m[3][2] + m_[3] * m[3][3];
    return out;
  }

  ALGTEM const mat4<T> mat4<T>::operator*(const mat4<T> &m) const{
    mat4<T> out;
    out[0] = m_[0] * m[0][0] + m_[1] * m[0][1] + m_[2] * m[0][2] + m_[3] * m[0][3];
    out[1] = m_[0] * m[1][0] + m_[1] * m[1][1] + m_[2] * m[1][2] + m_[3] * m[1][3];
    out[2] = m_[0] * m[2][0] + m_[1] * m[2][1] + m_[2] * m[2][2] + m_[3] * m[2][3];
    out[3] = m_[0] * m[3][0] + m_[1] * m[3][1] + m_[2] * m[3][2] + m_[3] * m[3][3];
    return out;
  }

  ALGTEM vec4<T> mat4<T>::operator*(const vec4<T> &v){
    vec4<T> out;
    const T v0{v[0]}, v1{v[1]}, v2{v[2]}, v3{v[3]};
    out[0] = m_[0][0] * v0 + m_[1][0] * v1 + m_[2][0] * v2 + m_[3][0] * v3;
    out[1] = m_[0][1] * v0 + m_[1][1] * v1 + m_[2][1] * v2 + m_[3][1] * v3;
    out[2] = m_[0][2] * v0 + m_[1][2] * v1 + m_[2][2] * v2 + m_[3][2] * v3;
    out[3] = m_[0][3] * v0 + m_[1][3] * v1 + m_[2][3] * v2 + m_[3][3] * v3;
    return out;
  }

  ALGTEM const vec4<T> mat4<T>::operator*(const vec4<T> &v) const{
    vec4<T> out;
    const T v0{v[0]}, v1{v[1]}, v2{v[2]}, v3{v[3]};
    out[0] = m_[0][0] * v0 + m_[1][0] * v1 + m_[2][0] * v2 + m_[3][0] * v3;
    out[1] = m_[0][1] * v0 + m_[1][1] * v1 + m_[2][1] * v2 + m_[3][1] * v3;
    out[2] = m_[0][2] * v0 + m_[1][2] * v1 + m_[2][2] * v2 + m_[3][2] * v3;
    out[3] = m_[0][3] * v0 + m_[1][3] * v1 + m_[2][3] * v2 + m_[3][3] * v3;
    return out;
  }

  ALGTEM vec3<T> mat4<T>::operator*(const vec3<T> &v){
    vec3<T> out;
    const T v0{v[0]}, v1{v[1]}, v2{v[2]};
    out[0] = m_[0][0] * v0 + m_[1][0] * v1 + m_[2][0] * v2 + m_[3][0];
    out[1] = m_[0][1] * v0 + m_[1][1] * v1 + m_[2][1] * v2 + m_[3][1];
    out[2] = m_[0][2] * v0 + m_[1][2] * v1 + m_[2][2] * v2 + m_[3][2];
    return out;
  }

  ALGTEM const vec3<T> mat4<T>::operator*(const vec3<T> &v) const{
    vec3<T> out;
    const T v0{v[0]}, v1{v[1]}, v2{v[2]};
    out[0] = m_[0][0] * v0 + m_[1][0] * v1 + m_[2][0] * v2 + m_[3][0];
    out[1] = m_[0][1] * v0 + m_[1][1] * v1 + m_[2][1] * v2 + m_[3][1];
    out[2] = m_[0][2] * v0 + m_[1][2] * v1 + m_[2][2] * v2 + m_[3][2];
    return out;
  }

  ALGTEM mat4<T>& mat4<T>::operator+=(const mat4<T> &m){
    m_[0] += m[0];
    m_[1] += m[1];
    m_[2] += m[2];
    m_[3] += m[3];

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::operator+=(const T s){
    m_[0] += s;
    m_[1] += s;
    m_[2] += s;
    m_[3] += s;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::operator-=(const mat4<T> &m){
    m_[0] -= m[0];
    m_[1] -= m[1];
    m_[2] -= m[2];
    m_[3] -= m[3];

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::operator-=(const T s){
    m_[0] -= s;
    m_[1] -= s;
    m_[2] -= s;
    m_[3] -= s;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::operator*=(const mat4<T> &m){
    return *this = *this * m;
  }

  ALGTEM mat4<T>& mat4<T>::operator*=(const T s){
    m_[0] *= s;
    m_[1] *= s;
    m_[2] *= s;
    m_[3] *= s;

    return *this;
  }

  ALGTEM mat4<T>& mat4<T>::operator/=(const T s){
    m_[0] /= s;
    m_[1] /= s;
    m_[2] /= s;
    m_[3] /= s;

    return *this;
  }

  ALGTEM T* mat4<T>::data(){
    return &data_[0];
  }

  ALGTEM const T* mat4<T>::data() const{
    return &data_[0];
  }

  template class mat4<double>;
  template class mat4<float>;
  template class mat4<int>;
}
