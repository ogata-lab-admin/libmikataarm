#include <iostream>
#include <float.h>
#include "CppUTest/CommandLineTestRunner.h"

const static double eps = 1.0e-16;
TEST_GROUP(KinematicsTestGroup) {
 


  TEST_SETUP() {

  }
 
  TEST_TEARDOWN() {

  }
};

TEST(KinematicsTestGroup, TestDouble) {
  DOUBLES_EQUAL(1.0, 1.0, eps);
}




int main(int argc, char** argv) {
  return RUN_ALL_TESTS(argc, argv);
}

