#ifndef COBOT_H
#define COBOT_H

#ifdef RB_EXPORTS
#define COBOT_DECLSPEC __declspec(dllexport)
#else
#define COBOT_DECLSPEC __declspec(dllimport)
#endif

#include "rb-common-header.h"
#include "rb-point.h"
#include "rb-joint.h"

using namespace std;

#include <thread>
#include <WinSock2.h>

#pragma comment(lib, "ws2_32")

namespace rb {
	
	//프로그램 모드
	enum class PG_MODE {
		SIMULATION,
		REAL
	};

	//원동작 타입
	enum class CIRCLE_TYPE {
		INTENDED,
		CONSTANT,
		RADIAL
	};

	//디지털 출력 전압
	enum class DOUT_VOLT {
		VOLT_0,
		VOLT_12,
		VOLT_24
	};
	
	//협동로봇 상태
	enum class COBOT_STATUS {
		IDLE,
		PAUSED,
		RUNNING,
		UNKNOWN
	};	

	//제어박스 디지털 출력 신호
	const int SIGNAL_BYPASS = -1;
	const int SIGNAL_LOW = 0;
	const int SIGNAL_HIGH = 1;

	//제어박스 디지털 출력 포트
	using DOUT_PORT = struct dout_port{
		int d0 = SIGNAL_BYPASS;		int d1 = SIGNAL_BYPASS; 		int d2 = SIGNAL_BYPASS; 		int d3 = SIGNAL_BYPASS;
		int d4 = SIGNAL_BYPASS;		int d5 = SIGNAL_BYPASS; 		int d6 = SIGNAL_BYPASS; 		int d7 = SIGNAL_BYPASS;
		int d8 = SIGNAL_BYPASS;		int d9 = SIGNAL_BYPASS; 		int d10 = SIGNAL_BYPASS; 		int d11 = SIGNAL_BYPASS;
		int d12 = SIGNAL_BYPASS;		int d13 = SIGNAL_BYPASS; 		int d14 = SIGNAL_BYPASS; 		int d15 = SIGNAL_BYPASS;
	};

	class COBOT_DECLSPEC Cobot
	{
	public:
		Cobot();
		virtual ~Cobot();

		//제어박스와 통신 연결
		bool ConnectToCB(string ip="10.0.2.7");

		string __Version();

		//협동로봇 초기화
		bool CobotInit();

		//협동로봇 프로그램 모드 설정
		bool SetProgramMode(PG_MODE mode);

		//현재 로봇의 관절 각도값 획득
		Joint GetCurrentJoint();
		tuple<float, float, float, float, float, float> GetCurrentSplitedJoint();

		//현재 로봇의 TCP 위치 획득
		Point GetCurrentTCP();
		tuple<float, float, float, float, float, float> GetCurrentSplitedTCP();

		//현재 로봇의 관절 각도 값, TCP 위치 획득
		tuple<Joint, Point> GetCurrentJP();
		
		//협동로봇 속도 설정
		bool SetBaseSpeed(float spd);

		//협동로봇 일시정지
		bool MotionPause();

		//협동로봇 정지
		bool MotionHalt();

		//협동로봇 일시정지 재개
		bool MotionResume();

		//협동로봇 충돌정지 재개
		bool CollisionResume();		

		//Move Linear
		bool MoveL(Point p, float spd, float acc);
		bool MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc);
		
		//Move Joint
		bool MoveJ(Joint j, float spd, float acc);
		bool MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);

		//Move Joint Blend
		bool MoveJB_Clear();
		bool MoveJB_Add(Joint j, float spd, float acc);
		bool MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);
		bool MoveJB_Run();		

		//Move Linear Blend
		bool MoveLB_Clear();
		bool MoveLB_Add(Point p, float spd, float acc, float radius);
		bool MoveLB_Add(float x, float y, float z, float rx, float ry, float rz, float spd, float acc, float radius);
		bool MoveLB_Run();

		//원 동작
		bool MoveCircle_ThreePoint(float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc, CIRCLE_TYPE type);\
		bool MoveCircle_ThreePoint(Point p1, Point p2, float spd, float acc, CIRCLE_TYPE type);

		//제어박스 디지털 출력
		bool ControlBoxDigitalOut(DOUT_PORT port);
		bool ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15);
		
		//제어박스 아날로그 출력
		bool ControlBoxAnalogOut(float a0, float a1, float a2, float a3);

		//툴플렌지 출력
		bool ToolOut(int d0, int d1, DOUT_VOLT volt);		

		//default: 30ms
		void SetWaitTime(int millisec);

		//협동로봇 idle 유무
		bool IsIdle();

		//협동로봇 일시정지 유무
		bool IsPause();

		//협동로봇 현재 상태
		COBOT_STATUS GetCurrentCobotStatus();

	private:
		void ReadCmd();
		void ReadData();
		void RunThread();
		void ReqDataStart();

		bool SetMotionBreak(string condition, float dec_time);

		bool isValidIP(string ip);		

		bool socketCmdCom(string ip);
		bool socketCmdClose();

		
		bool socketDataCom(string ip);
		bool socketDataClose();

		Point current_tcp_;
		Joint current_joint_;

		string ip_address_;
		systemSTAT systemStat;
		systemCONFIG systemConfig;
		systemPOPUP  systemPopup;

		bool initFlag = false;
		bool cmdConfirmFlag = true;
		bool moveCmdFlag = false;
		int moveCmdCnt = 0;

		int CMD_CLIENT_FD_ = 0;
		int DATA_CLIENT_FD_ = 0;

		struct sockaddr_in  cmd_addr_;
		struct sockaddr_in  data_addr_;

		int wait_time_ = 30;

		bool bReadCmd = false;

		vector<char> recv_buf_;

		thread proc1;
		thread proc2;
		thread proc3;
	};
}



#endif
