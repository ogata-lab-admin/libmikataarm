#pragma once

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBSSR_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBSSR_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef mikata_EXPORTS
#define LIBSSR_API __declspec(dllexport)
#else
#ifdef EXPORT_mikata_STATIC
#define EXPORT_dynamixel_STATIC
#define LIBSSR_API 
#else
#define LIBSSR_API __declspec(dllimport)
#endif

#endif

#else // ifdef WIN32
#define LIBSSR_API 

#endif // ifdef WIN32

#include <vector>
#include <exception>
#include "DynamixelV2.h"

namespace ssr {
  namespace mikata {
    class MikataException : public std::exception {
    public:
      MikataException() {
	msg = "MikataException";
      }

      MikataException(const char* str) {
	msg = "MikataException:";
	msg += str;
      }
      virtual ~MikataException() throw() {}

      const char* what() const throw() {
	return msg.c_str();
      }

      std::string msg;
    };

    /**
     *
     */
    class RangeException : public MikataException {
    public:
      RangeException(const char* str) : MikataException() {
	msg = "RagneException:";
	msg += str;
      }
      virtual ~RangeException() throw() {}
    };


    class TimeoutException : public MikataException {
    public:
      TimeoutException(const char* str) : MikataException() {
	msg = "TimeoutException:";
	msg += str;
      }
      virtual ~TimeoutException() throw() {}
    };

    struct JointCommand {
    public:
      double angle;
      double velocity;
    public:
    JointCommand() : angle(0), velocity(0) {}
      void copyFrom(const JointCommand& cmd) {
	angle = cmd.angle;
	velocity = cmd.velocity;
      }

      JointCommand(const JointCommand& cmd) {
	copyFrom(cmd);
      }

      void operator=(const JointCommand& cmd) {
	copyFrom(cmd);
      }

      ~JointCommand() {}
    };

    struct JointInfo {
    public:
      double angle;
    public:
    JointInfo() : angle(0) {}
    JointInfo(double a): angle(a) {}
      ~JointInfo() {}

    public:
      JointInfo(const JointInfo& i) {
	copyFrom(i);
      }

      void copyFrom(const JointInfo& i) {
	angle = i.angle;
      }

      void operator=(const JointInfo& i) {
	copyFrom(i);
      }

    };

	

    struct LimitValue {
      double upper;
      double lower;
      LimitValue() : upper(0), lower(0) {}
      LimitValue(double u, double l): upper(u), lower(l) {}
      ~LimitValue() {}
      LimitValue(const LimitValue& lv) {
	copyFrom(lv);
      }
	
      void copyFrom(const LimitValue& lv) {
	upper = lv.upper;
	lower = lv.lower;
      }

      void operator=(const LimitValue& lv) {
	copyFrom(lv);
      }

    };

    static const uint32_t numJoints = 6;

    /**
     *
     */
    class MikataArm {
    private:
      uint8_t m_IDs[numJoints];
      uint8_t m_GripperID;

      double m_JointOffset[numJoints];
      double m_GripperOffset;
      ssr::dynamixel::DynamixelV2 m_Dynamixel;      


      LimitValue m_JointLimits[numJoints];
      LimitValue m_GripperLimit;
    public: 
      MikataArm(const char* filename, const uint32_t baudrate);
      virtual ~MikataArm() ;

    public:
      /**
       * Get JointInformation
       * @returns Vector of JointInfo struct
       * @see JointInfo
       */
      std::vector<JointInfo> jointInfos();

      /**
       * Set Servo ON or OFF of All Motors (Except Gripper)
       * @param flag Servo ON if true, else Servo OFF.
       */
      void servoOn(const bool flag = true);

      /**
       * Go to the Zero position of Arm
       */
      void goHome();

      /**
       * Get JOint Information of Gripper
       * @return JointInfo struct
       * @see JointInfo
       */
      JointInfo gripper();

      /**
       * Set Servo ON/OFF of Gripper
       * @param flag If true, Servo ON, else Servo OFF
       */
      void gripperServoOn(const bool flag = true);

    public:
      std::vector<LimitValue> getJointLimits() const;

      void setJointLimits(std::vector<LimitValue>& lvs);
      
      LimitValue getGripperLimit() const;

      void setGripperLimit(const LimitValue& lv);

      /**
       * Returns Current Electric Current. [mA]
       */
      std::vector<double> getJointCurrent();

      void setJointCurrent(const std::vector<double>& currentMA);
      
    public:
      void move(const std::vector<JointCommand>& cmds);

      void moveGripper(const JointCommand& cmd);

      void waitAttained(const long timeoutMS);
      
      void waitGripperAttained(const long timeoutMS);

      void openGripper();
      void closeGripper();
      void moveGripper(const double ratio);

      void setVelocityRatio(const double ratio);

	  void setAccelerationRatio(const double ratio);
    };
  };
};
