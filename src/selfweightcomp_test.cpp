
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <thread>

#include "mikata.h"
#include "kinematics.h"
#include "Thread.h"
using namespace ssr::mikata;


const std::vector<double> test_joints{0, 0, M_PI/2, 0, 0, 0};
std::vector<double> js;

void comp_mikata(MikataArm& m) {
  std::vector<JointCommand> jcs;
  for(int i = 0;i < 6;i++) {
    JointCommand jc;
    jc.angle = test_joints[i];
    jc.velocity = 0.01;
    jcs.push_back(jc);
  }
  m.move(jcs);
  std::cout << "Waiting 3sec..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3));


  int c;
  std::cout << "Now switching to Free joint mode. PLEASE SUPPORT THE ROBOT ARM and press enter" << std::endl;
  std::cin >> c;
  for(int i = 0;i < 6;i++) {
    m.servoOn(i, false);
    m.currentMode(i, true);
    m.servoOn(i, true);
  }


  bool endflag = false;
  std::thread thread([&endflag]() {
		       std::string buf;
		       while(true) {
			 std::cout << "Input 'r' and 'Enter' to exit" << std::endl;
			 std::cin >> buf;
			 if (buf[0] == 'r') {
			   endflag = true;
			   break;
			 }
		       }
		     });

  std::cout << "Now Self weight comp mode is activated." << std::endl;
  
  while(!endflag) {
    std::vector<JointInfo> jis = m.jointInfos();
    for(int i = 0;i < numJoints;i++) {
      js.push_back(jis[i].angle);
    }
    
    auto ks = forward_kinematics_mat(js, 0);
    SelfWeightCompParam param;
    auto current = selfWeightComp(param, ks);
    for(int i = 0;i < numJoints;i++) {
      m.setTargetCurrent(i, current[i]);
    }
  }

  std::cout << "Now switching to Free joint mode. PLEASE SUPPORT THE ROBOT ARM and press enter" << std::endl;
  std::cin >> c;
  for(int i = 0;i < 6;i++) {
    m.servoOn(i, false);
    m.currentMode(i, false);
    m.servoOn(i, true);
  }

  thread.join();
}


int main(const int argc, const char* argv[]) {
  if (argc != 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./selfweightcomp_test filename baudrate" << std::endl;
    return -1;
  }
  
  try {
    MikataArm m(argv[1], atoi(argv[2]));
    comp_mikata(m, 1);

  } catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
