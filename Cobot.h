#ifndef COBOT_H
#define COBOT_H


#ifdef RB_EXPORTS
#define COBOT_DECLSPEC __declspec(dllexport)
#else
#define COBOT_DECLSPEC __declspec(dllimport)
#endif

#include "CommonHeader.h"
#include "Point.h"
#include "Joint.h"

using namespace std;

// Thread
#include <thread>
// Socket
#include <WinSock2.h>

#pragma comment(lib, "ws2_32")

namespace rb {
	//프로그램 모드
	enum class PG_MODE {
		SIMULATION,
		REAL
	};

	//회전 옵션
	enum class CIRCLE_TYPE {
		INTENDED,
		CONSTANT,
		RADIAL
	};

	enum class DOUT_VOLT {
		VOLT_0,
		VOLT_12,
		VOLT_24
	};

	class COBOT_DECLSPEC Cobot
	{
	public:
		Cobot();
		virtual ~Cobot();
		bool ConnectToCB(string ip="10.0.2.7");
		string __Version();

		bool CobotInit();
		bool SetProgramMode(PG_MODE mode);

		Joint GetCurrentJoint();
		tuple<float, float, float, float, float, float> GetCurrentSplitedJoint();

		Point GetCurrentTCP();
		tuple<float, float, float, float, float, float> GetCurrentSplitedTCP();
		
		bool MoveL(Point p, float spd, float acc);
		bool MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc);
		
		bool MoveJ(Joint j, float spd, float acc);
		bool MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);

		bool MoveJB_Clear();
		bool MoveJB_Add(Joint j, float spd, float acc);
		bool MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);
		bool MoveJB_Run();		

		bool MoveLB_Clear();
		bool MoveLB_Add(Point p, float spd, float acc, float radius);
		bool MoveLB_Add(float x, float y, float z, float rx, float ry, float rz, float spd, float acc, float radius);
		bool MoveLB_Run();

		bool MoveCircle_ThreePoint(float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc, CIRCLE_TYPE type);\
		bool MoveCircle_ThreePoint(Point p1, Point p2, float spd, float acc, CIRCLE_TYPE type);

		bool ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15);
		bool ControlBoxAnalogOut(float a0, float a1, float a2, float a3);
		bool ToolOut(int d0, int d1, DOUT_VOLT volt);

		bool TestPlay();
		bool TestStop();

		bool BaseSpeedChange(float spd);

		bool MotionPause();
		bool MotionHalt();
		bool MotionResume();
		bool CollisionResume();

		bool SetMotionBreak(string condition, float dec_time);
		//void ProgramMode_Real();
		void ReadCmd();
		void ReadData();

		void ReqDataStart();

		void RunThread();

		//default: 30ms
		void SetProcessingPeriod(int millisec);
		//void JoinThread();
	private:
		bool isValidIP(string ip);
		bool isMotionIdle();

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

		int processing_period_ = 30;

		bool bReadCmd = false;

		vector<char> recv_buf_;

		thread proc1;
		thread proc2;
		thread proc3;
	};
}



#endif
