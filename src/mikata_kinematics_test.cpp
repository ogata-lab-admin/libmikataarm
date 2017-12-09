
#include <stdlib.h>
#include <iostream>
#include "mikata.h"
#include "kinematics.h"
#include "Thread.h"
using namespace ssr::mikata;

int main(const int argc, const char* argv[]) {
  if (argc < 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }
  
  if (argc == 3) {
    try {
      MikataArm m(argv[1], atoi(argv[2]));
      m.servoOn(false);
      // m.goHome();
      std::vector<double> joints;    
      double joint_offset[6] = {
	0, 0, 0, 0, 0, 0 };
	//	0, -M_PI/2 + 1.42923183, -1.42923183, 0, 0, 0 };
      for(int j = 0;j < 1000;j++) {
	std::vector<JointInfo> js = m.jointInfos();
	JointInfo g = m.gripper();
	joints.clear();
	std::cout << "------------------------------" << std::endl;
	for(int i = 0;i < numJoints;i++) {
	  std::cout << "j[" << i << "]" << js[i].angle - joint_offset[i] << std::endl;
	  joints.push_back(js[i].angle - joint_offset[i]);
	}
	std::cout << "gripper:" << g.angle << std::endl;
	Matrix44 m = forward_kinematics(joints);
	std::cout << "P" << std::endl << str(m) << std::endl;

	std::vector<double> solved = inverse_kinematics(m);
	for(int i = 0;i < numJoints;i++) {
	  std::cout << "j[" << i << "]" << solved[i] << std::endl;
	}
	std::cout << "------------------------------" << std::endl;
	std::cout << "------------------------------" << std::endl;

	for(int i = 0;i < numJoints;i++) {
	  std::cout << "d[" << i << "]" << fabs((js[i].angle - joint_offset[i]) - solved[i]) << std::endl;
	}


	ssr::Thread::Sleep(100);
      }
    
    } catch (std::exception& ex) {
      std::cout << "Exception: " << ex.what() << std::endl;
      return -1;
    }

  } else if (argc == 6) {


    try {

      double x = atof(argv[3]);
      double y = atof(argv[4]);
      double z = atof(argv[5]);
      
      std::string buf;
      std::cout << "X, Y, Z = " << x << ", " << y << ", " << z << std::endl;
      std::cout << "OK?" << std::ends;
      std::cin >> buf;
      std::cout << "buf = " << buf << std::endl;
      if (buf.c_str()[0] != 'y' && buf.c_str()[0] != 'Y') {
	return -1;
      }
      MikataArm m(argv[1], atoi(argv[2]));      
      m.servoOn(true);
      //      m.goHome();


      
      Matrix44 mm;
      mm.v[0][0] = -1.0; mm.v[0][1] = 0; mm.v[0][2] =  0.0; mm.v[0][3] = x;
      mm.v[1][0] =  0.0; mm.v[1][1] = 1; mm.v[1][2] =  0.0; mm.v[1][3] = y;
      mm.v[2][0] =  0.0; mm.v[2][1] = 0; mm.v[2][2] = -1.0; mm.v[2][3] = z;
      mm.v[3][0] =  0.0; mm.v[3][1] = 0; mm.v[3][2] =  0.0; mm.v[3][3] = 1;
      std::vector<double> solved = inverse_kinematics(mm);
      std::vector<JointCommand> jcs;
      for(int i = 0;i < numJoints;i++) {
	std::cout << "j[" << i << "]" << solved[i] << std::endl;
	JointCommand jc;
	jc.angle = solved[i];

	jcs.push_back(jc);
      }
      m.move(jcs);

      std::vector<double> joints;
      ssr::Thread::Sleep(3000);      
      std::vector<JointInfo> js = m.jointInfos();      
      std::cout << "------------------------------" << std::endl;
      for(int i = 0;i < numJoints;i++) {
	std::cout << "j[" << i << "]=" << js[i].angle << std::endl;
	joints.push_back(js[i].angle);
      }
      Matrix44 m2 = forward_kinematics(joints);
      std::cout << "P" << std::endl << str(m2) << std::endl;
      m.servoOn(false);
      
      //      mm.v[0] = 0.0; mm.v[1]
      /*
      
      std::vector<double> joints;    
      double joint_offset[6] = {
	0, -M_PI/2 + 1.42923183, -1.42923183, 0, 0, 0 };
      for(int j = 0;j < 1000;j++) {
	std::vector<JointInfo> js = m.jointInfos();
	joints.clear();
	std::cout << "------------------------------" << std::endl;
	for(int i = 0;i < numJoints;i++) {
	  std::cout << "j[" << i << "]" << js[i].angle - joint_offset[i] << std::endl;

	  joints.push_back(js[i].angle - joint_offset[i]);

	}
	Matrix44 m = forward_kinematics(joints);
	std::cout << "P" << std::endl << str(m) << std::endl;

	std::vector<double> solved = inverse_kinematics(m);
	for(int i = 0;i < numJoints;i++) {
	  std::cout << "j[" << i << "]" << solved[i] << std::endl;
	}
	std::cout << "------------------------------" << std::endl;
	std::cout << "------------------------------" << std::endl;

	for(int i = 0;i < numJoints;i++) {
	  std::cout << "d[" << i << "]" << fabs((js[i].angle - joint_offset[i]) - solved[i]) << std::endl;
	}


	ssr::Thread::Sleep(100);
      }
      */
    } catch (std::exception& ex) {
      std::cout << "Exception: " << ex.what() << std::endl;
      return -1;
    }

  }

  return 0;
}
