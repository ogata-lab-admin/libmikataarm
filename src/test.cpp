
#include <stdlib.h>
#include <iostream>
#include "mikata.h"
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
    
    for(int j = 0;j < 1000;j++) {
      std::vector<JointInfo> js = m.jointInfos();
      for(int i = 0;i < numJoints;i++) {
	std::cout << "j[" << i << "]" << js[i].angle << std::endl;
      }
      ssr::Thread::Sleep(100);
    }
    
  } catch (std::exception& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
    return -1;
  }

  return 0;
}
