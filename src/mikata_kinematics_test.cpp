
#include <stdlib.h>
#include <iostream>
#include "mikata.h"
#include "kinematics.h"
#include "Thread.h"
using namespace ssr::mikata;

int main(const int argc, const char* argv[]) {
  if (argc != 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }
  
  try {
    MikataArm m(argv[1], atoi(argv[2]));
    m.servoOn(false);
    // m.goHome();
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
    
  } catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
