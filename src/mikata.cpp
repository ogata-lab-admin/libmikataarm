#include <iostream>
#include "mikata.h"

//#define DEBUG

/**
 * この値は，Offsetの値が適用された後の制限値です．
 * なので，この値の次元の角度ですべて0になった場合は，
 * アームはまっすぐ上に伸びている状態になるはずです．
 */
static const double jointLimitValues[][2] = {
  {-M_PI/2, M_PI/2},
  {-M_PI/2, M_PI/2},
  {-M_PI/2, M_PI/2},
  {-M_PI/2, M_PI/2},
  {-M_PI/2, M_PI/2},
  {-M_PI/2, M_PI/2}
};

static const double gripperLimitValue[2] = {-M_PI/2, M_PI};

using namespace ssr::mikata;
using namespace ssr::dynamixel;

MikataArm::MikataArm(const char* filename, const uint32_t baudrate) :m_Dynamixel(filename, baudrate) {
  m_IDs[0] = 1;
  m_IDs[1] = 2;
  m_IDs[2] = 3;
  m_IDs[3] = 4;
  m_IDs[4] = 5;
  m_IDs[5] = 6;

  for(int i = 0;i < numJoints;i++) {
    m_JointLimits[i].lower = jointLimitValues[i][0];
    m_JointLimits[i].upper = jointLimitValues[i][1];
  }

  m_GripperID = 7;

  m_GripperLimit.lower = gripperLimitValue[0];
  m_GripperLimit.upper = gripperLimitValue[1];

  m_JointOffset[0] = M_PI;
  m_JointOffset[1] = M_PI;
  m_JointOffset[2] = M_PI;
  m_JointOffset[3] = M_PI;
  m_JointOffset[4] = M_PI;
  m_JointOffset[5] = M_PI;
  
  m_GripperOffset = M_PI;
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

JointInfo MikataArm::gripper() {
  int32_t position = m_Dynamixel.GetCurrentPosition(m_GripperID);
#ifdef DEBUG
  std::cout << "mikata_gripper: " << position << std::endl;
  std::cout << "[";
  for(int j = 0;j < 4;j++) {
    std::cout << (int)(uint8_t)((position >> (8*j)) & 0x00FF) << " ";
  }
  std::cout << "]" << std::endl;
#endif
  double angle = pos_to_rad(position) - m_GripperOffset;
  JointInfo info;
  info.angle = angle;
  return info;
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

void MikataArm::gripperServoOn(const bool flag) {
  if (flag) {
    m_Dynamixel.TorqueEnable(m_GripperID);
  } else {
    m_Dynamixel.TorqueDisable(m_GripperID);
  }
}

void MikataArm::goHome() {
  for(int i = 0;i < numJoints;i++) {
    int32_t pos = rad_to_pos(0.0 + m_JointOffset[i]);
    m_Dynamixel.MovePosition(m_IDs[i], pos);
  }
}

std::vector<LimitValue> MikataArm::getJointLimits() const {
  std::vector<LimitValue> lvs;
  for(int i = 0;i < numJoints;i++) {
    lvs.push_back(m_JointLimits[i]);
  }
  return lvs;
}

void MikataArm::setJointLimits(std::vector<LimitValue>& lvs) {
  if (lvs.size() != numJoints) {
    throw MikataException("setJointLimits Error. Invalid size of passed argument lvs::std::vector<LimitValue>");
  }
  
  for(int i = 0;i < numJoints;i++) {
    m_JointLimits[i] = lvs[i];
  }
}

LimitValue MikataArm::getGripperLimit() const {
  return m_GripperLimit;
}

void MikataArm::setGripperLimit(const LimitValue& lv) {
  m_GripperLimit = lv;
}


void MikataArm::move(const std::vector<JointCommand>& cmds) {
  for(int i = 0;i < numJoints;i++) {
    double angle = cmds[i].angle;
    if (angle > m_JointLimits[i].upper) angle = m_JointLimits[i].upper;
    else if (angle < m_JointLimits[i].lower) angle = m_JointLimits[i].lower;

    m_Dynamixel.MovePosition(m_IDs[i], rad_to_pos(angle + m_JointOffset[i]));
  }
}


void MikataArm::moveGripper(const JointCommand& cmd) {
  double angle = cmd.angle > m_GripperLimit.upper ? m_GripperLimit.upper : (cmd.angle < m_GripperLimit.lower ? m_GripperLimit.lower : cmd.angle);
  m_Dynamixel.MovePosition(m_GripperID, rad_to_pos(angle + m_GripperOffset));
}



