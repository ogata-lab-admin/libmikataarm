#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <string>
#include <sstream>
#include <vector>

//typedef double Matrix44[4][4];
struct Matrix44 {
public:
  double v[4][4];
public:
  Matrix44() {
    for(int i = 0;i < 4;i++)
      for(int j = 0;j < 4;j++) 
	v[i][j] = 0;
  }
  ~Matrix44() {}

public:
  void copyFrom(const Matrix44& m) {
    for(int i = 0;i < 4;i++)
      for(int j = 0;j < 4;j++) 
	this->v[i][j] = m.v[i][j];
  }

  Matrix44(const Matrix44& m) {
    copyFrom(m);
  }

  void operator=(const Matrix44& m) {
    copyFrom(m);
  }

};


struct Vector3 {
  double v[3];

  Vector3() {
    v[0] = v[1] = v[2] = 0;
  }

  Vector3(const Vector3& v3) {
    v[0] = v3.v[0]; v[1] = v3.v[1]; v[2] = v3.v[2];
  }

  void operator=(const Vector3& v3) {
    v[0] = v3.v[0]; v[1] = v3.v[1]; v[2] = v3.v[2];
  }


};
 

inline std::string str(const Matrix44& m) {
  std::stringstream ss;
  ss << m.v[0][0] << " " << m.v[0][1] << " " << m.v[0][2] << " " << m.v[0][3] << "\n"
     << m.v[1][0] << " " << m.v[1][1] << " " << m.v[1][2] << " " << m.v[1][3] << "\n"
     << m.v[2][0] << " " << m.v[2][1] << " " << m.v[2][2] << " " << m.v[2][3] << "\n"
     << m.v[3][0] << " " << m.v[3][1] << " " << m.v[3][2] << " " << m.v[3][3];
  return ss.str();
}


class KinematicsState {
public:
  virtual ~KinematicsState() {}
};

Matrix44 jointMat(const std::shared_ptr<KinematicsState>& ks, const int index);
Matrix44 jointMatAbs(const std::shared_ptr<KinematicsState>& ks, const int index);

std::shared_ptr<KinematicsState> forward_kinematics_mat(const std::vector<double>& joints, int debug);
Matrix44 forward_kinematics(const std::vector<double>& pos, int debug = 0);
std::vector<double> inverse_kinematics(const Matrix44& mat);

inline Vector3 MatrixToEulerZYZ(const Matrix44& hg) {
  Vector3 v;
  double c2 = hg.v[2][2];
  double e = 1.0e-5;
  if (fabs(c2 - 1.0) < e) {
    v.v[0] = 0.0;
    v.v[1] = 0.0;
    //v.v[2] = atan2(hg.v[1][2], hg.v[0][2]);
    double d = hg.v[0][0] + hg.v[1][1];
    v.v[2] = acos(d/2);
    return v;
  }

  v.v[0] = atan2(hg.v[1][2], hg.v[0][2]);  
  v.v[1] = acos(c2);
  //  double s2 = sin(v.v[1]);
  //  double s3 = hg.v[2][1] / s2;
  //  double c3 = (hg.v[2][0] - c2*s3)/(-s2);
  //  v.v[2] = atan2(s3, c3);
  v.v[2] = atan2(hg.v[2][1], -hg.v[2][0]);
  return v;
}


/**
 * 自重補償のためのパラメータ
 */
class SelfWeightCompParam {
public:
  double m_[3]; // 各リンクの重量中心の大きさ [kg]
  double l_[3]; // 重量中心から屈曲軸までの長さ[mm]
  double K_[4];     // トルクと電流値とのゲイン [milliAmp / (kg mm)]
  SelfWeightCompParam() {
    m_[0] = 80;
    m_[1] = 160;
    m_[2] = 170; // [g]

    l_[0] = 100;
    l_[1] = 80;
    l_[2] = 50;;

    K_[0] = 0.01;
    K_[1] = 0.01;
    K_[2] = 0.01;
    K_[3] = 0.01;    
  }
};


std::vector<double> selfWeightComp(const SelfWeightCompParam& param, const std::shared_ptr<KinematicsState>& kinematics);

inline std::string str(const Vector3& v) {
  std::stringstream ss;
  ss << "[[" << v.v[0] << "]\n"
     << " [" << v.v[1] << "]\n"
     << " [" << v.v[2] << "]";
  return ss.str();
}



