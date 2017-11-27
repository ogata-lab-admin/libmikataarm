#include <iostream>
#include "mikata.h"

//#define DEBUG

using namespace ssr::mikata;
using namespace ssr::dynamixel;

MikataArm::MikataArm(const char* filename, const uint32_t baudrate) :m_Dynamixel(filename, baudrate) {
  m_IDs[0] = 1;
  m_IDs[1] = 2;
  m_IDs[2] = 3;
  m_IDs[3] = 4;
  m_IDs[4] = 5;
  m_IDs[5] = 6;

  m_JointOffset[0] = M_PI;
  m_JointOffset[1] = M_PI;
  m_JointOffset[2] = M_PI;
  m_JointOffset[3] = M_PI;
  m_JointOffset[4] = M_PI;
  m_JointOffset[5] = M_PI;
}


MikataArm::~MikataArm() {
}

std::vector<JointInfo> MikataArm::jointInfos() {
  std::vector<JointInfo> joints;
  for(int i = 0;i < 6;i++) {
    int32_t position = m_Dynamixel.GetCurrentPosition(m_IDs[i]);
#ifdef DEBUG
    std::cout << "mikata:" << i << ": " << position << std::endl;
    std::cout << "[";
    for(int j = 0;j < 4;j++) {
      std::cout << (int)(uint8_t)((position >> (8*j)) & 0x00FF) << " ";
    }
    std::cout << "]" << std::endl;
#endif
    double angle = pos_to_rad(position) - m_JointOffset[i];
    joints.push_back(JointInfo(angle));

  }

  return joints;
}


void MikataArm::servoOn(const bool flag) {
  for(int i = 0;i < numJoints;i++) {
    if (flag) {
      m_Dynamixel.TorqueEnable(m_IDs[i]);
    } else {
      m_Dynamixel.TorqueDisable(m_IDs[i]);
    }
  }
}

void MikataArm::goHome() {
  for(int i = 0;i < numJoints;i++) {
    int32_t pos = rad_to_pos(0.0 + m_JointOffset[i]);
    m_Dynamixel.MovePosition(m_IDs[i], pos);
  }
}





