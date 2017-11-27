#pragma once

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBSSR_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBSSR_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef LIBMIKATA_EXPORTS
#define LIBSSR_API __declspec(dllexport)
#else
#define LIBSSR_API __declspec(dllimport)
#endif

#else // ifdef WIN32
#define LIBSSR_API 

#endif // ifdef WIN32

#include <vector>

#include "DynamixelV2.h"

namespace ssr {
  namespace mikata {


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

	

    static const uint32_t numJoints = 6;

    class MikataArm {


    private:
      uint8_t m_IDs[numJoints];
      double m_JointOffset[numJoints];
      ssr::dynamixel::DynamixelV2 m_Dynamixel;      
    public: 
      MikataArm(const char* filename, const uint32_t baudrate);
      virtual ~MikataArm() ;

    public:
      std::vector<JointInfo> jointInfos();

      void servoOn(const bool flag = true);
      void goHome();
    };
  };
};
