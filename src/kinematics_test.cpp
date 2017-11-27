#include <math.h>
#include <iostream>
#include "kinematics.h"


int main(void) {
  std::cout << "Kinematics Test Start" << std::endl;
  int test_count = 0;
  std::vector<double> joints;
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  joints.push_back(0);
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  Matrix44 m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;

  joints[0] = M_PI/2;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;


  joints[0] = 0;
  joints[2] = M_PI/2;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;


  joints[0] = 0;
  joints[2] = M_PI/2;
  joints[4] = M_PI/2;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;

  joints[0] = 0;
  joints[2] = -M_PI/2;
  joints[4] = M_PI/2;
  joints[5] = M_PI/4;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::cout << "ZYZ:" << std::endl << str(MatrixToEulerZYZ(m)) << std::endl;


  joints[0] = 0;
  joints[2] = M_PI/2;
  joints[4] = M_PI/4;
  joints[5] = M_PI/4;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::cout << "ZYZ:" << std::endl << str(MatrixToEulerZYZ(m)) << std::endl;


  joints[0] = 0;
  joints[2] = M_PI/2;
  joints[3] = M_PI/4;
  joints[4] = M_PI/4;
  joints[5] = M_PI/4;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::cout << "ZYZ:" << std::endl << str(MatrixToEulerZYZ(m)) << std::endl;

  joints[0] = M_PI/6;
  joints[2] = M_PI/2;
  joints[3] = M_PI/4;
  joints[4] = M_PI/4;
  joints[5] = M_PI/4;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::cout << "ZYZ:" << std::endl << str(MatrixToEulerZYZ(m)) << std::endl;


  joints[0] = M_PI/6;
  joints[1] = M_PI/6;
  joints[2] = M_PI/2;
  joints[3] = 0;
  joints[4] = M_PI/5;
  joints[5] = -M_PI/4;
  std::cout << "Joints:";
  for(int i = 0;i < 6;i++) {
    std::cout << joints[i] << " ";
  }
  std::cout << "\n";
  m = forward_kinematics(joints);
  std::cout << "Test[" << test_count++ <<"]" << std::endl << str(m) << std::endl;
  std::cout << "ZYZ:" << std::endl << str(MatrixToEulerZYZ(m)) << std::endl;



  return 0;
}
