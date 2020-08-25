
#include <stdlib.h>
#include <iostream>
#include <cstdlib>
#include <thread>

#include "mikata.h"
#include "kinematics.h"
#include "Thread.h"
using namespace ssr::mikata;


void help() {

  std::cout << "Command list:" << std::endl;
  std::cout << "  o: SERVO ON" << std::endl;
  std::cout << "  p: SERVO OFF" << std::endl;
  std::cout << "  io*: SERVO ON for joint index * [0-5]" << std::endl;
  std::cout << "  ip*: SERVO ON for joint index * [0-5]" << std::endl;
  std::cout << "  s: Show Joint Angles" << std::endl;
  
  std::cout << "  q: Quit" << std::endl;

}

std::vector<std::vector<double> > test_joints = {
						  {0, 0, 0, 0, 0, 0},
						  {0, 0, M_PI/2, 0, 0, 0},
						  {0, 0, M_PI/2, M_PI/2, M_PI/2, 0},
						  {0, 0, M_PI/2, -M_PI/2, M_PI/2, 0},
						  {0, 0, 0, 0, M_PI/2, 0},
						  {0, 0, 0, M_PI/2, M_PI/2, 0},
						  {0, 0, M_PI/2, M_PI/2, -M_PI/2, 0}
};



void comp_mikata(MikataArm& m, int n) {
  std::vector<JointCommand> jcs;
  for(int i = 0;i < 6;i++) {
    JointCommand jc;
    jc.angle = test_joints[n][i];
    jc.velocity = 0.01;
    jcs.push_back(jc);
  }
  m.move(jcs);
  std::cout << "Waiting 3sec..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3));


  bool endflag = false;
  std::thread thread([&endflag]() {
		       std::string buf;
		       while(true) {

			 std::cin >> buf;
			 if (buf[0] == 'r') {
			   endflag = true;
			   break;
			 }
		       }
		     });


  m.servoOn(0, false);
  m.currentMode(0, true);
  m.servoOn(0, true);
  m.servoOn(1, false);
  m.currentMode(1, true);
  m.servoOn(1, true);
  m.servoOn(2, false);
  m.currentMode(2, true);
  m.servoOn(2, true);
  m.servoOn(3, false);
  m.currentMode(3, true);
  m.servoOn(3, true);
  m.servoOn(4, false);
  m.currentMode(4, true);
  m.servoOn(4, true);  
  m.servoOn(5, false);
  m.currentMode(5, true);
  m.servoOn(5, true);
  
  while(!endflag) {
    std::vector<double> js;
    std::vector<JointInfo> jis = m.jointInfos();
    for(int i = 0;i < numJoints;i++) {
      js.push_back(jis[i].angle);
    }

    
    
    auto ks = forward_kinematics_mat(js, 0);
    SelfWeightCompParam param;
    auto current = selfWeightComp(param, ks);
    for(int i = 0;i < numJoints;i++) {
      std::cout << "j[" << i << "]" << jis[i].current << "/(" << current[i] << ")" << std::endl;
      m.setTargetCurrent(i, current[i]);
    }
    

  }

  m.servoOn(0, false);
  m.currentMode(0, false);
  m.servoOn(0, true);
  m.servoOn(1, false);
  m.currentMode(1, false);
  m.servoOn(1, true);
  m.servoOn(2, false);
  m.currentMode(2, false);
  m.servoOn(2, true);
  m.servoOn(3, false);
  m.currentMode(3, false);
  m.servoOn(3, true);
  m.servoOn(4, false);
  m.currentMode(4, false);
  m.servoOn(4, true);  
  m.servoOn(5, false);
  m.currentMode(5, false);
  m.servoOn(5, true);
  

  thread.join();

}

void test_mikata(MikataArm& m, int n) {
  std::vector<JointCommand> jcs;
  for(int i = 0;i < 6;i++) {
    JointCommand jc;
    jc.angle = test_joints[n][i];
    jc.velocity = 0.01;
    jcs.push_back(jc);
  }
  m.move(jcs);

  std::cout << "Waiting 3sec..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  std::vector<JointInfo> jis = m.jointInfos();
  std::vector<double> js;
  for(int i = 0;i < numJoints;i++) {
    std::cout << "Jangle[" << i << "]" << jis[i].angle << std::endl;
    js.push_back(jis[i].angle);
  }
  auto fk = forward_kinematics(js, 0);
  std::cout << "Forward Kinematics:" << std::endl;
  std::cout << str(fk) << std::endl;

  auto ks = forward_kinematics_mat(js, 0);
  SelfWeightCompParam param;
  auto current = selfWeightComp(param, ks);
  for(int i = 0;i < numJoints;i++) {
    std::cout << "j[" << i << "]" << jis[i].current << "/(" << current[i] << ")" << std::endl;
  }
  
}


int main(const int argc, const char* argv[]) {
  if (argc != 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./selfweightcomp_test filename baudrate" << std::endl;
    return -1;
  }
  
  try {
    MikataArm m(argv[1], atoi(argv[2]));
    bool endflag = false;
    while(!endflag) {
      std::string buf;
      std::cout << "Input Command (help: h):" << std::endl;
      if(!std::getline(std::cin, buf)) {
	continue;
      }


      switch(buf[0]) {
      case 'h':
	help();
	break;

      case 'o':
	m.servoOn(true);
	break;

      case 'p':
	m.servoOn(false);
	break;

      case 'i':
	{
	  char c = buf[1];
	  char index = buf[2];
	  if (c == 'o') {
	    m.servoOn(index - '0', true);
	  } else if (c == 'p') {
	    m.servoOn(index - '0', false);
	  }
	}
	break;
      case '0':
	test_mikata(m, 0);
	break;

      case '1':
	test_mikata(m, 1);
	break;

      case '2':
	test_mikata(m, 2);
	break;

      case '3':
	test_mikata(m, 3);
	break;

      case '4':
	comp_mikata(m, 1);
	break;
	
      case '5':
	test_mikata(m, 4);
	break;

      case '6':
	test_mikata(m, 5);
	break;

      case '7':
	test_mikata(m, 6);
	break;
	
      case 's':
	{
	  std::vector<JointInfo> jis = m.jointInfos();
	  for(int i = 0;i < numJoints;i++) {
	    std::cout << "j[" << i << "]" << jis[i].angle << " [rad]" << std::endl;
	    std::cout << "j[" << i << "]" << jis[i].current << " [mA]" << std::endl;
	  }
	}
	break;
	

      case 'S':
	{
	  std::vector<JointInfo> jis = m.jointInfos();
	  std::vector<double> js;
	  for(int i = 0;i < numJoints;i++) {
	    //	    std::cout << "j[" << i << "]" << jis[i].angle << std::endl;

	    js.push_back(jis[i].angle);
	  }

	  auto ks = forward_kinematics_mat(js, 0);
	  SelfWeightCompParam param;
	  auto current = selfWeightComp(param, ks);
	  for(int i = 0;i < numJoints;i++) {
	    //	    std::cout << "j[" << i << "]" << jis[i].current << std::endl;	    	    
	    std::cout << "j[" << i << "]" << jis[i].current << "/(" << current[i] << ")" << std::endl;
	  }
	  
	  //	  std::cout << "Forward Kinematics:" << std::endl;
	  //	  std::cout << str(fk) << std::endl;
	}
	break;
	

      case 'q':
	endflag = true;
	break;


      }

      
    }
  } catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
