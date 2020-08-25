#include <stdlib.h>
#include <iostream>
#include "mikata.h"
#include "Thread.h"

using namespace ssr::mikata;



void measure(MikataArm& arm) {
  

}

int main(const int argc, const char* argv[]) {

  if (argc != 3) {
    std::cout << "Invalid Usage." << std::endl;
    std::cout << "USAGE: $./test filename baudrate" << std::endl;
    return -1;
  }
  
  try {
    MikataArm m(argv[1], atoi(argv[2]));
    m.servoOn(false);

    while(true) {
      int c;
      std::cin >> c;
      if (c == 'b') m.servoOn(true);
      
      if (c == 'm') measure(m);
      
      if (c == 'e') break;
    }

  } catch (std::exception &e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return -1;
  } return 0;
