//
//  LinearAlgebra.cpp
//  qd
//
//  Created by Edward Janne on 10/18/23.
//

#include "LinearAlgebra.hpp"
#include <utility>
#include <math.h>

Vector3 Vector3::zero(0.0, 0.0, 0.0);
Vector3 Vector3::i(1.0, 0.0, 0.0);
Vector3 Vector3::j(0.0, 1.0, 0.0);
Vector3 Vector3::k(0.0, 0.0, 1.0);

Vector3::Vector3(float x, float y, float z)
: v { x, y, z }
{ }

Vector3::Vector3(const float *iVector)
: v { iVector[0], iVector[1], iVector[2] }
{ }

Vector3::Vector3(const Vector3 &iOther)
: v { iOther[0], iOther[1], iOther[2] }
{ }
    
float Vector3::operator[](int i) const {
    return v[i];
}

float &Vector3::operator[](int i) {
    return v[i];
}

Vector3 &Vector3::operator=(const Vector3 &iVec) {
    int i = 3;
    while(i--) v[i] = iVec[i];
    return *this;
}
    
Vector3 &Vector3::operator+=(const Vector3 &iVec) {
    int i = 3;
    while(i--) v[i] += iVec[i];
    return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &iVec) {
    int i = 3;
    while(i--) v[i] -= iVec[i];
    return *this;
}

Vector3 &Vector3::operator*=(float iScal) {
    int i = 3;
    while(i--) v[i] *= iScal;
    return *this;
}

Vector3 &Vector3::operator/=(float iScal) {
    int i = 3;
    float r = 1.0 / iScal;
    while(i--) v[i] *= r;
    return *this;
}

Vector3 Vector3::operator+(const Vector3 &iVec) const {
    Vector3 result(*this);
    result += iVec;
    return result;
}

Vector3 Vector3::operator-(const Vector3 &iVec) const {
    Vector3 result(*this);
    result -= iVec;
    return result;
}

Vector3 Vector3::operator*(float iScal) const {
    Vector3 result(*this);
    result *= iScal;
    return result;
}

Vector3 Vector3::operator/(float iScal) const {
    Vector3 result(*this);
    result /= iScal;
    return result;
}

Vector3 Vector3::operator-() const {
  Vector3 result(*this);
  result *= -1.0;
  result[3] = 1.0;
  return result;
}
    
Vector3 Vector3::operator!() const {
    Vector3 result(*this);
    result /= sqrtf(*this * *this);
    return result;
}

float Vector3::operator*(const Vector3 &iVec) const {
    return v[0] * iVec[0] + v[1] * iVec[1] + v[2] * iVec[2];
}

Vector3 Vector3::operator/(const Vector3 &iVec) const {
    return Vector3(v[1] * iVec[2] - v[2] * iVec[1], v[2] * iVec[0] - v[0] * iVec[2], v[0] * iVec[1] - v[1] * iVec[0]);
}
    
Vector3 Vector3::operator>(const Vector3 &iVec) const {
    return iVec * ((*this * iVec) / (iVec * iVec));
}

Vector3 Vector3::operator^(const Vector3 &iVec) const {
    return *this - (*this > iVec);
}

//----------------------------------------------------------------------------

Vector4 Vector4::zero(0.0, 0.0, 0.0);
Vector4 Vector4::i(1.0, 0.0, 0.0);
Vector4 Vector4::j(0.0, 1.0, 0.0);
Vector4 Vector4::k(0.0, 0.0, 1.0);

Vector4::Vector4(float x, float y, float z, float w)
: v { x, y, z, w }
{ }

Vector4::Vector4(const float *iVector, float w)
: v { iVector[0], iVector[1], iVector[2], w }
{ }

Vector4::Vector4(const Vector4 &iOther)
: v { iOther[0], iOther[1], iOther[2], iOther[3] }
{ }
    
float Vector4::operator[](int i) const {
    return v[i];
}

float &Vector4::operator[](int i) {
    return v[i];
}

Vector4 &Vector4::operator=(const Vector4 &iVec) {
    int i = 4;
    while(i--) v[i] = iVec[i];
    return *this;
}
    
Vector4 &Vector4::operator+=(const Vector4 &iVec) {
    int i = 4;
    while(i--) v[i] += iVec[i];
    return *this;
}

Vector4 &Vector4::operator-=(const Vector4 &iVec) {
    int i = 4;
    while(i--) v[i] -= iVec[i];
    return *this;
}

Vector4 &Vector4::operator*=(float iScal) {
    int i = 4;
    while(i--) v[i] *= iScal;
    return *this;
}

Vector4 &Vector4::operator/=(float iScal) {
    int i = 4;
    float r = 1.0 / iScal;
    while(i--) v[i] *= r;
    return *this;
}

Vector4 Vector4::operator+(const Vector4 &iVec) const {
    Vector4 result(*this);
    result += iVec;
    return result;
}

Vector4 Vector4::operator-(const Vector4 &iVec) const {
    Vector4 result(*this);
    result -= iVec;
    return result;
}

Vector4 Vector4::operator*(float iScal) const {
    Vector4 result(*this);
    result *= iScal;
    return result;
}

Vector4 Vector4::operator/(float iScal) const {
    Vector4 result(*this);
    result /= iScal;
    return result;
}

Vector4 Vector4::operator-() const {
  Vector4 result(*this);
  result *= -1.0;
  result[3] = 1.0;
  return result;
}
    
Vector4 Vector4::operator!() const {
    Vector4 result(*this);
    result /= sqrtf(*this * *this);
    return result;
}

float Vector4::operator*(const Vector4 &iVec) const {
    return v[0] * iVec[0] + v[1] * iVec[1] + v[2] * iVec[2];
}

Vector4 Vector4::operator/(const Vector4 &iVec) const {
    return Vector4(v[1] * iVec[2] - v[2] * iVec[1], v[2] * iVec[0] - v[0] * iVec[2], v[0] * iVec[1] - v[1] * iVec[0]);
}
    
Vector4 Vector4::operator>(const Vector4 &iVec) const {
    return iVec * ((*this * iVec) / (iVec * iVec));
}

Vector4 Vector4::operator^(const Vector4 &iVec) const {
    return *this - (*this > iVec);
}

Matrix3 Matrix3::operator*(const Matrix3 &iMat) const {
    Matrix3 result(0);
    
    int i = 9;
    while(i--) {
        int x = i % 3;
        int y = i / 3;
        int j = 3;
        while(j--) {
            result[y * 3 + x] += m[y * 3 + j] * iMat[j * 3 + x];
        }
    }
    return result;
}

Vector3 Matrix3::operator*(const Vector3 &iVec) const {
    Vector3 result;
    
    int y = 3;
    while(y--) {
        int x = 3;
        while(x--) {
            result[y] += m[y * 3 + x] * iVec[x];
        }
    }
    
    return result;
}

Matrix4::Matrix4(Init iMode, const Vector4 &iValue, float iAngle)
: m { 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0 }
{
    int i = 16;
    if(iMode != zero) {
      while(i--) {
        m[i] = ((iMode && (i % 5 == 0)) ? 1.0 : 0.0);
      }
    }
    switch(iMode) {
      case rotation: {
          float s = sinf(iAngle);
          float c = cosf(iAngle);
          float cc = (1.0 - c);
          float x = iValue[0], y = iValue[1], z = iValue[2],
                xs = x * x, ys = y * y, zs = z * z;
          m[0] = c + xs * cc;
          m[1] = x * y * cc - z * s;
          m[2] = x * z * cc + y * s;
          m[3] = 0.0;
          m[4] = x * y * cc + z * s;
          m[5] = c + ys * cc;
          m[6] = y * z * cc - x * s;
          m[7] = 0.0;
          m[8] = x * z * cc - y * s;
          m[9] = y * z * cc + x * s;
          m[10] = c + zs * cc;
          m[11] = 0.0;
          m[12] = 0.0;
          m[13] = 0.0;
          m[14] = 0.0;
          m[15] = 1.0;
        }
        break;
      case translation:
        i = 3;
        while(i--) {
          m[i * 4 + 3] = iValue[i];
        }
        break;
      case scale:
        i = 3;
        while(i--) {
            m[i*5] *= iValue[i];
        }
        break;
      default:
        break;
    }
}

Matrix4::Matrix4(const float *iMatrix)
: m { 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0 }
{
    int i = 16;
    while(i--) {
        m[i] = iMatrix[i];
    }
}

Matrix4::Matrix4(const Matrix4 &iOther) {
  int i = 16;
  while(i--) {
    m[i] = iOther[i];
  }
}

float Matrix4::operator[](int i) const {
    return m[i];
}

float &Matrix4::operator[](int i) {
    return m[i];
}
    
Matrix4 Matrix4::operator*(const Matrix4 &iMat) const {
    Matrix4 result(zero);
    
    int i = 16;
    while(i--) {
        int x = i % 4;
        int y = i / 4;
        int j = 4;
        while(j--) {
            result[y * 4 + x] += m[y * 4 + j] * iMat[j * 4 + x];
        }
    }
    return result;
}

Vector4 Matrix4::operator*(const Vector4 &iVec) const {
    Vector4 result;
    
    int y = 4;
    while(y--) {
        int x = 4;
        while(x--) {
            result[y] += m[y * 4 + x] * iVec[x];
        }
    }
    
    return result;
}
