#include "kinematics.h"
#include <vector>
#include <iostream>

static const double L1 = 25+80.5;
static const double L2 = 168.4;
static const double L3 = 24;
static const double L4 = 150;
static double L5 = 88+112.04;


#include <Eigen/Core>
//#include <Eigen/SVD>
//#include <Eigen/LU>

inline double normalize_angle(double angle) {
  while(angle > M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

typedef Eigen::Matrix<double, 4, 4> MATRIX44;
typedef Eigen::Matrix<double, 6, 6> MATRIX66;
typedef Eigen::Vector4d VECTOR4;
typedef Eigen::Matrix<double, 6, 1> VECTOR6;


static MATRIX44 RotTransX(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  //result(0, 0) = 1; result(0, 1) = 0; result(0, 2) = 0;
  /* result(1, 0) = 0;*/ result(1, 1) = c; result(1, 2) =-s;
  /* result(2, 0) = 0;*/ result(2, 1) = s; result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}


static MATRIX44 RotTransY(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; /*result(0, 1) = 0;*/ result(0, 2) = s;
  //result(1, 0) = 0; result(1, 1) = 1; result(1, 2) = 0; 
  result(2, 0) =-s; /*result(2, 1) = 0;*/ result(2, 2) = c;
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;
  return result;
}

static MATRIX44 RotTransZ(const double t, const double x, const double y, const double z) {
  MATRIX44 result = MATRIX44::Identity(4,4);
  double c = cos(t); double s = sin(t);
  result(0, 0) = c; result(0, 1) = -s; /*result(0, 2) = 0;*/
  result(1, 0) = s; result(1, 1) = c; /*result(1, 2) = 0;*/
  //result(2, 0) = 0; result(2, 1) = 0; result(2, 2) = 1; 
  result(0, 3) = x; result(1, 3) = y; result(2, 3) = z;

  return result;
}


Matrix44 mat_to_mat(const MATRIX44& mat) {
  Matrix44 m;
  for(int i = 0;i < 4;i++) {
    for(int j = 0;j < 4;j++) {
      m.v[i][j] = mat(i, j);
    }
  }
  
  return m;
}


MATRIX44 mat_to_mat(const Matrix44& mat) {
  MATRIX44 m;
  for(int i = 0;i < 4;i++) {
    for(int j = 0;j < 4;j++) {
      m(i, j) = mat.v[i][j];
    }
  }
  return m;
}

Matrix44 forward_kinematics(const std::vector<double> joints, int debug) {
  MATRIX44 jointMat[6];
  jointMat[0] = RotTransZ(joints[0], 0, 0, 0);
  jointMat[1] = RotTransY(joints[1], 0, 0, L1);
  jointMat[2] = RotTransY(joints[2], L3, 0, L2);
  jointMat[3] = RotTransZ(joints[3], 0, 0, L4);
  jointMat[4] = RotTransY(joints[4], 0, 0, 0);
  jointMat[5] = RotTransZ(joints[5], 0, 0, L5);

  MATRIX44 jointMatAbs[6];
  jointMatAbs[0] = jointMat[0];
  for(int i = 0;i < 5;i++) {
    jointMatAbs[i+1] = jointMatAbs[i] * jointMat[i+1];
    if (debug) {
      std::cout << "J" << i+1 << std::endl << jointMatAbs[i+1] << std::endl;
    }
  }
  VECTOR4 vec(0, 0, -200.040, 1);
  if (debug) {
    std::cout << "test:" << std::endl << jointMatAbs[5] * vec << std::endl;
    inverse_kinematics(mat_to_mat(jointMatAbs[5]));
		       
  }
  return mat_to_mat(jointMatAbs[5]);
}

std::vector<double> inverse_kinematics(const Matrix44& mat) {
  MATRIX44 m = mat_to_mat(mat);
  VECTOR4  v(0, 0, -L5, 1);
  VECTOR4  p5 = m * v;
  VECTOR4  p2(0, 0, L1, 0);
  VECTOR4  d25 = p5 - p2;
  double   D25 = d25.norm();
  double th_offst = 1.42923183;
  double L23 = 170.1;
  double th3 = M_PI*6/4 - acos((L23*L23 + L4*L4 - D25*D25) / 2 / L23 / L4) - th_offst;
  double th1 = atan2(p5(1), p5(0));
  double th2 = M_PI/2 - atan2(p5(2)-L1, sqrt(p5(0)*p5(0)+p5(1)*(p5(1))))
    - acos((L23*L23 + D25*D25 - L4*L4) / 2 / L23 / D25) - (M_PI/2 - th_offst);
  
  
  MATRIX44 mm = RotTransY(-th3, 0, 0, 0) * RotTransY(-th2, 0, 0, 0) * RotTransZ(-th1, 0, 0, 0) * m;
  std::cout << "mm = "<< std::endl << mm << std::endl;
  Vector3 vv = MatrixToEulerZYZ(mat_to_mat(mm));
  std::cout << "vv = " << std::endl << str(vv) << std::endl;
  std::vector<double> j;
  j.push_back(th1);
  j.push_back(th2);
  j.push_back(th3);
  j.push_back(vv.v[0]);
  j.push_back(vv.v[1]);
  j.push_back(vv.v[2]);
  return j;
}
