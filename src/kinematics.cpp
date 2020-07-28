#include "kinematics.h"
#include <vector>
#include <iostream>

static const double L1 = 25+80.5;
static const double L2 = 168.4;
static const double L3 = 24;
static const double L4 = 150;
static const double L5 = 88+112.04;
static const double L23 = 170.1;

#include <Eigen/Core>
#include <Eigen/Dense>
//#include <Eigen/SVD>
//#include <Eigen/LU>

inline double normalize_angle(double angle) {
  while(angle > M_PI) angle -= 2 * M_PI;
  while(angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

typedef Eigen::Matrix<double, 4, 4> MATRIX44;
typedef Eigen::Matrix<double, 6, 6> MATRIX66;
typedef Eigen::Vector3d VECTOR3;
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

class KinematicsStateImpl : public KinematicsState {
public:
  std::vector<double> jointAngles;
  std::vector<MATRIX44> jointMat;
  std::vector<MATRIX44> jointMatAbs;
};

Matrix44 jointMat(const std::shared_ptr<KinematicsState>& ks, const int index) {
  return mat_to_mat(std::dynamic_pointer_cast<KinematicsStateImpl>(ks)->jointMat[index]);
}


Matrix44 jointMatAbs(const std::shared_ptr<KinematicsState>& ks, const int index) {
  return mat_to_mat(std::dynamic_pointer_cast<KinematicsStateImpl>(ks)->jointMatAbs[index]);
}


std::shared_ptr<KinematicsState> forward_kinematics_mat(const std::vector<double>& joints, int debug) {
  auto ks = std::make_shared<KinematicsStateImpl>();
  ks->jointAngles = joints;
  double L23 = 170.1;
  ks->jointMat.emplace_back(RotTransZ(joints[0], 0, 0, 0));
  ks->jointMat.emplace_back(RotTransY(joints[1], 0, 0, L1));
  ks->jointMat.emplace_back(RotTransY(joints[2], 0, 0, L23));
  ks->jointMat.emplace_back(RotTransZ(joints[3], 0, 0, L4));
  ks->jointMat.emplace_back(RotTransY(joints[4], 0, 0, 0));
  ks->jointMat.emplace_back(RotTransZ(joints[5], 0, 0, L5));

  MATRIX44 jointMatAbs[6];
  jointMatAbs[0] = ks->jointMat[0];
  ks->jointMatAbs.push_back(ks->jointMat[0]);
  for(int i = 0;i < 5;i++) {
    ks->jointMatAbs.push_back(ks->jointMatAbs[i] * ks->jointMat[i+1]);
    if (debug) {
      std::cout << "J" << i+1 << std::endl << ks->jointMatAbs[i+1] << std::endl;
    }
  }
  VECTOR4 vec(0, 0, -200.040, 1);
  if (debug) {
    std::cout << "test:" << std::endl << ks->jointMatAbs[5] * vec << std::endl;
    //    inverse_kinematics(mat_to_mat(ks->jointMatAbs[5]));
  }
  
  return ks;
}

Matrix44 forward_kinematics(const std::vector<double>& joints, int debug) {
  /*
  MATRIX44 jointMat[6];
  double L23 = 170.1;
  jointMat[0] = RotTransZ(joints[0], 0, 0, 0);
  jointMat[1] = RotTransY(joints[1], 0, 0, L1);
  jointMat[2] = RotTransY(joints[2], 0, 0, L23);
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
    //inverse_kinematics(mat_to_mat(jointMatAbs[5]));
		       
  }
  */
  auto ks = forward_kinematics_mat(joints, debug);
  return jointMatAbs(ks, 5);
  //  return mat_to_mat(jointMatAbs[5]);
}

std::vector<double> inverse_kinematics(const Matrix44& mat) {
  MATRIX44 m = mat_to_mat(mat);
  VECTOR4  v(0, 0, -L5, 1);
  VECTOR4  p5 = m * v;
  VECTOR4  p2(0, 0, L1, 0);
  VECTOR4  d25 = p5 - p2;
  // norm method uses float type...
  double   D25 = sqrt(d25(0)*d25(0) + d25(1)*d25(1) + d25(2)*d25(2));//d25.norm();
  double   e = 1.0e-0;
  //double th_offst = 1.42923183;

  
  double th3;
  std::cout << "--D25wow:" << D25 << "/" << L23+L4<< std::endl;    
  if (fabs(D25 - (L23+L4)) < 1.0e-5) {
    th3 = 0;
  } else {
    th3 = M_PI - acos((L23*L23 + L4*L4 - D25*D25) / 2 / L23 / L4);// - th_offst;
  }

  double th1;
  if (fabs(p5(0)) > e) {
    th1 = atan2(p5(1), p5(0));
  } else {
    th1 = 0;
  }

  double th2;
  if (fabs(D25 - (L23+L4)) < 1.0e-10) {
    th2 = atan2(sqrt(p5(0)*p5(0)+p5(1)*(p5(1))), p5(2)-L1);//- (M_PI/2/* - th_offst*/);
  } else {
    th2 = atan2(sqrt(p5(0)*p5(0)+p5(1)*(p5(1))), p5(2)-L1)
      - acos((L23*L23 + D25*D25 - L4*L4) / 2 / L23 / D25);// - (M_PI/2/* - th_offst*/);
  }
  
  
  MATRIX44 mm = RotTransY(-th3, 0, 0, 0) * RotTransY(-th2, 0, 0, 0) * RotTransZ(-th1, 0, 0, 0) * m;
  //  std::cout << "mm = "<< std::endl << mm << std::endl;
  Vector3 vv = MatrixToEulerZYZ(mat_to_mat(mm));
  //  std::cout << "vv = " << std::endl << str(vv) << std::endl;
  std::vector<double> j;
  j.push_back(th1);
  j.push_back(th2);
  j.push_back(th3);
  j.push_back(vv.v[0]);
  j.push_back(vv.v[1]);
  j.push_back(vv.v[2]);
  return j;
}


std::vector<double> selfWeightComp(const SelfWeightCompParam& param, const std::shared_ptr<KinematicsState>& kinematics) {

  auto ks = std::dynamic_pointer_cast<KinematicsStateImpl>(kinematics);
  VECTOR4 M3posRel{0, 0, param.l_[2], 1};
  VECTOR4 M2posRel{0, 0, param.l_[1], 1};
  VECTOR4 M1posRel{0, 0, param.l_[0], 1};
  
  auto abs4 = ks->jointMatAbs[4];
  auto abs3 = ks->jointMatAbs[3];  
  auto abs2 = ks->jointMatAbs[2];
  auto abs1 = ks->jointMatAbs[1];  
  auto positionJ4 = abs4.block<4,1>(0,3);
  auto positionJ2 = abs2.block<4,1>(0,3);  
  auto positionJ1 = abs1.block<4,1>(0,3);
  
  auto positionM3Abs = abs4 * M3posRel; // 手首重心絶対座標系位置
  auto positionM2Abs = abs2 * M2posRel; // 手首重心絶対座標系位置
  auto positionM1Abs = abs1 * M1posRel; // 手首重心絶対座標系位置  
  
  auto rM3_J4 = (positionM3Abs - positionJ4).block<3,1>(0,0);
  auto rM3_J2 = (positionM3Abs - positionJ2).block<3,1>(0,0);
  auto rM2_J2 = (positionM2Abs - positionJ2).block<3,1>(0,0);
  auto rM3_J1 = (positionM3Abs - positionJ1).block<3,1>(0,0);
  auto rM2_J1 = (positionM2Abs - positionJ1).block<3,1>(0,0);
  auto rM1_J1 = (positionM1Abs - positionJ1).block<3,1>(0,0);    

  auto rotJ4 = abs4.block<3,3>(0,0);
  auto rotJ3 = abs3.block<3,3>(0,0);
  auto rotJ2 = abs2.block<3,3>(0,0);
  auto rotJ1 = abs1.block<3,3>(0,0);
  
  //std::cout << "rM3_J4:" << rM3_J4 << std::endl;

  //  std::cout << "torque_M3_J4ABS:" << torque_M3_J4ABS << std::endl;
  VECTOR3 torque_M3_J4ABS = VECTOR3{0, 0, -param.m_[2]}.cross(rM3_J4);
  VECTOR3 torque_M3_J4 = rotJ4.transpose() * torque_M3_J4ABS;

  VECTOR3 torque_M3_J3 = rotJ3.transpose() * torque_M3_J4ABS;


  VECTOR3 torque_M3_J2ABS = rM3_J2.cross(VECTOR3{0, 0, param.m_[2]});
  VECTOR3 torque_M3_J2 = rotJ2.transpose() * torque_M3_J2ABS;
  VECTOR3 torque_M2_J2ABS = rM2_J2.cross(VECTOR3{0, 0, param.m_[1]});
  VECTOR3 torque_M2_J2 = rotJ2.transpose() * torque_M2_J2ABS;


  VECTOR3 torque_M3_J1ABS = rM3_J1.cross(VECTOR3{0, 0, param.m_[2]});
  VECTOR3 torque_M3_J1 = rotJ1.transpose() * torque_M3_J1ABS;
  VECTOR3 torque_M2_J1ABS = rM2_J1.cross(VECTOR3{0, 0, param.m_[1]});
  VECTOR3 torque_M2_J1 = rotJ1.transpose() * torque_M2_J1ABS;
  VECTOR3 torque_M1_J1ABS = rM1_J1.cross(VECTOR3{0, 0, param.m_[0]});
  VECTOR3 torque_M1_J1 = rotJ1.transpose() * torque_M1_J1ABS;



  //  std::cout << "torque_M3_J3:" << torque_M3_J3 << std::endl;  
  //  std::cout << "torque_M3_J2:" << torque_M3_J2 << std::endl;
  //double rM2_J2 = param.l_[1] * sin(ks->jointAngles[1] + ks->jointAngles[2]);



  //auto abs5 = jointMatAbs(kinematics, 4);
  //VECTOR3 positionJ5(abs5.v[0][3], abs5.v[1][3], abs5.v[2][3]);
  //VECTOR3 positionM3{param.l_[2] * abs5.v[0][2] + abs5.v[0][3],
  //param.l_[2] * abs5.v[1][2] + abs5.v[1][3],
  //param.l_[2] * abs5.v[2][2] + abs5.v[2][3]
  //};
  //VECTOR3 rM3_J5 = positionM3-positionJ5;
  //VECTOR3 torque_M3_J5ABS = rM3_J5.cross(VECTOR3{0, 0, param.m_[2]});


  
  return {0,
	  param.K_[0]*(torque_M3_J1[1] + torque_M2_J1[1] + torque_M1_J1[1]),
	  param.K_[1]*(torque_M3_J2[1]+torque_M2_J2[1]),
	  param.K_[2]*torque_M3_J3[2],
	  param.K_[3]*torque_M3_J4[1],
	  0};
}
