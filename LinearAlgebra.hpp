//
//  LinearAlgebra.hpp
//  qd
//
//  Created by Edward Janne on 10/18/23.
//

#ifndef LinearAlgebra_hpp
#define LinearAlgebra_hpp

#include <stdio.h>
#include <math.h>
#include <Arduino.h>

typedef struct Vector3 {
    static Vector3 zero;
    static Vector3 i;
    static Vector3 j;
    static Vector3 k;
    
    float v[3];
    
    Vector3(float x = 0.0, float y = 0.0, float z = 0.0);
    Vector3(const float *iVector);
    Vector3(const Vector3 &iOther);
    
    float operator[](int i) const;
    float &operator[](int i);
    
    Vector3 &operator=(const Vector3 &iVec);
    
    Vector3 &operator+=(const Vector3 &iVec);
    Vector3 &operator-=(const Vector3 &iVec);
    Vector3 &operator*=(float iScal);
    Vector3 &operator/=(float iScal);
    
    Vector3 operator+(const Vector3 &iVec) const;
    Vector3 operator-(const Vector3 &iVec) const;
    Vector3 operator*(float iScal) const;
    Vector3 operator/(float iScal) const;

    Vector3 operator-() const;
    
    Vector3 operator!() const;                      // Normalized
    
    float operator*(const Vector3 &iVec) const;     // Dot product
    Vector3 operator/(const Vector3 &iVec) const;   // Cross product
    
    Vector3 operator>(const Vector3 &iVec) const;   // Projection
    Vector3 operator^(const Vector3 &iVec) const;   // Perpendicular
    
    float magnitude() const {
      return sqrtf(*this * *this);
    }
    
    void print() const {
      for(int i = 0; i < 3; i++) {
        if(i) Serial.print(", ");
        Serial.print(v[i]);
      }
      Serial.println();
    }
} Vector3;

typedef struct Vector4 {
    static Vector4 zero;
    static Vector4 i;
    static Vector4 j;
    static Vector4 k;
    
    float v[4];
    
    Vector4(float x = 0.0, float y = 0.0, float z = 0.0, float w = 0.0);
    Vector4(const float *iVector, float w = 0.0);
    Vector4(const Vector4 &iOther);
    
    float operator[](int i) const;
    float &operator[](int i);
    
    Vector4 &operator=(const Vector4 &iVec);
    
    Vector4 &operator+=(const Vector4 &iVec);
    Vector4 &operator-=(const Vector4 &iVec);
    Vector4 &operator*=(float iScal);
    Vector4 &operator/=(float iScal);
    
    Vector4 operator+(const Vector4 &iVec) const;
    Vector4 operator-(const Vector4 &iVec) const;
    Vector4 operator*(float iScal) const;
    Vector4 operator/(float iScal) const;

    Vector4 operator-() const;
    
    Vector4 operator!() const;                      // Normalized
    
    float operator*(const Vector4 &iVec) const;     // Dot product
    Vector4 operator/(const Vector4 &iVec) const;   // Cross product
    
    Vector4 operator>(const Vector4 &iVec) const;   // Projection
    Vector4 operator^(const Vector4 &iVec) const;   // Perpendicular
    
    float magnitude() const {
      return sqrtf(*this * *this);
    }
    
    void print() const {
      for(int i = 0; i < 4; i++) {
        if(i) Serial.print(", ");
        Serial.print(v[i]);
      }
      Serial.println();
    }
} Vector4;

typedef struct Matrix2 {
  float m[4];

  Matrix2(float a = 0.0, float b = 0.0, float c = 0.0, float d = 0.0) {
    m[0] = a;
    m[1] = b;
    m[2] = c;
    m[3] = d;
  }

  Matrix2(const Matrix2 &iOther) {
    for(int i = 0; i < 4; i++) {
      m[i] = iOther[i];
    }
  }

  float determinant() {
    return m[0] * m[3] - m[1] * m[2];
  }

  float operator[](int iIndex) const {
    return m[iIndex % 4];
  }

  float &operator[](int iIndex) {
    return m[iIndex % 4];
  }

  void print() {
    for(int y = 0; y < 2; y++) {
      for(int x = 0; x < 2; x++) {
        Serial.print(" ");
        Serial.print(m[y * 2 + x]);
      }
      Serial.println();
    }
  }
} Matrix2;

typedef struct Matrix3 {
  float m[9];

  Matrix3() {
    for(int i = 0; i < 9; i++) {
      m[i] = 0.0;
    }
  }

  Matrix3(const float *iMatrix) {
    for(int i = 0; i < 9; i++) {
      m[i] = iMatrix[i];
    }
  }

  Matrix3(const Matrix3 &iOther) {
    for(int i = 0; i < 9; i++) {
      m[i] = iOther[i];
    }
  }

  Matrix3 &operator *=(float iFactor) {
    for(int i = 0; i < 9; i++) {
      m[i] *= iFactor;
    }

    return *this;
  }
  
  Matrix3 operator *(const Matrix3 &iMat) const;
  Vector3 operator *(const Vector3 &iVec) const;
  Matrix3 operator *(float iFactor) const {
    Matrix3 sm(*this);
    sm *= iFactor;
    return sm;
  }

  Matrix3 &operator /=(float iFactor) {
    for(int i = 0; i < 9; i++) {
      m[i] /= iFactor;
    }

    return *this;
  }
  
  Matrix3 operator /(float iFactor) const {
    Matrix3 sm(*this);
    sm /= iFactor;
    return sm;
  }

  float operator[](int iIndex) const {
    return m[iIndex % 9];
  }

  float &operator[](int iIndex) {
    return m[iIndex % 9];
  }

  Matrix2 subMatrix(int iIndex) {
    Matrix2 sub;
    int x = iIndex % 3;
    int y = iIndex / 3;
    int n = 0;
    for(int j = 0; j < 3; j++) {
      if(j != y) {
        for(int i = 0; i < 3; i++) {
          if(i != x) {
            sub[n] = m[j * 3 + i];
            n++;
          }
        }
      }
    }
    return sub;
  }

  float determinant() {
    float negate = 1.0;
    float det = 0.0;
    for(int i = 0; i < 3; i++) {
      det += m[i] * subMatrix(i).determinant() * negate;
      negate *= -1.0;
    }
    return det;
  }

  Matrix3 cofactorMatrix() {
    Matrix3 cm;
    float det = 0.0;
    for(int i = 0; i < 9; i++) {
      cm[i] = subMatrix(i).determinant() * pow(-1.0, (float) (i/3 + i%3));
    }
    return cm;
  }

  Matrix3 transpose() {
    Matrix3 tm;
    for(int i = 0; i < 9; i++) {
      tm[i] = m[(i % 3) * 3 + (i / 3)];
    }
    return tm;
  }

  Matrix3 inverse() {
    return cofactorMatrix().transpose() / determinant();
  }

  void print() {
    for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
        Serial.print(" ");
        Serial.print(m[y * 3 + x]);
      }
      Serial.println();
    }
  }
} Matrix3;

typedef struct Matrix4 {
  typedef enum Init {
    zero = 0,
    identity = 1,
    rotation = 2,
    translation = 3,
    scale = 4
  } Init;

  float m[16];
  
  Matrix4(Init iMode = identity, const Vector4 &iAxis = Vector4::zero, float iAngle = 0.0);
  Matrix4(const float *iMatrix);
  Matrix4(const Matrix4 &iOther);

  Matrix4 &operator *=(float iFactor) {
    for(int i = 0; i < 16; i++) {
      m[i] *= iFactor;
    }

    return *this;
  }
  
  Matrix4 operator *(const Matrix4 &iMat) const;
  Vector4 operator *(const Vector4 &iVec) const;
  Matrix4 operator *(float iFactor) const {
    Matrix4 sm(*this);
    sm *= iFactor;
    return sm;
  }
  
  Matrix4 &operator /=(float iFactor) {
    for(int i = 0; i < 16; i++) {
      m[i] /= iFactor;
    }

    return *this;
  }
  
  Matrix4 operator /(float iFactor) const {
    Matrix4 sm(*this);
    sm /= iFactor;
    return sm;
  }

  float operator[](int i) const;
  float &operator[](int i);

  Matrix3 subMatrix(int iIndex) {
    Matrix3 sub;
    int x = iIndex % 4;
    int y = iIndex / 4;
    int n = 0;
    for(int j = 0; j < 4; j++) {
      if(j != y) {
        for(int i = 0; i < 4; i++) {
          if(i != x) {
            sub[n] = m[j * 4 + i];
            n++;
          }
        }
      }
    }
    return sub;
  }

  float determinant() {
    float negate = 1.0;
    float det = 0.0;
    for(int i = 0; i < 4; i++) {
      det += m[i] * subMatrix(i).determinant() * negate;
      negate *= -1.0;
    }
    return det;
  }

  Matrix4 cofactorMatrix() {
    Matrix4 cm;
    float negate = -1.0;
    float det = 0.0;
    for(int i = 0; i < 16; i++) {
      cm[i] = subMatrix(i).determinant() * pow(-1.0, (float) (i/4 + i % 4));
    }
    return cm;
  }

  Matrix4 transpose() {
    Matrix4 tm;
    for(int i = 0; i < 16; i++) {
      tm[i] = m[(i % 4) * 4 + (i / 4)];
    }
    return tm;
  }

  Matrix4 inverse() {
    return cofactorMatrix().transpose() / determinant();
  }

  void print() {
    for(int y = 0; y < 4; y++) {
      for(int x = 0; x < 4; x++) {
        Serial.print(" ");
        Serial.print(m[y * 4 + x]);
      }
      Serial.println();
    }
  }
} Matrix4;

#endif /* LinearAlgebra_hpp */
