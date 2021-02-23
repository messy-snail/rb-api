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
	enum class PG_MODE {
		SIMULATION,
		REAL
	};

	class COBOT_DECLSPEC Cobot
	{
	public:
		Cobot();
		virtual ~Cobot();
		bool ConnectToCB(string ip);
		string Version();

		void CobotInit();
		void SetProgramMode(PG_MODE mode);

		const Point GetCurrentPoint();
		
		void MoveL(Point p, float spd, float acc);
		void MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc);
		
		void MoveJ(Joint j, float spd, float acc);
		void MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);

		void MoveJB_Clear();
		void MoveJB_Add(Joint j, float spd, float acc);
		void MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc);
		void MoveJB_Run();		

		//void ProgramMode_Real();
		void ReadCmd();
		void ReadData();
	private:
		bool isValidIP(string ip);
		bool isMotionIdle();

		bool socketCmdCom(string ip);
		bool socketCmdClose();

		bool socketDataCom(string ip);
		bool socketDataClose();

		Point current_point_;
		Joint current_joint_;

		string ip_address_;
		systemSTAT systemStat;
		systemCONFIG systemConfig;
		systemPOPUP  systemPopup;

		bool initFlag = false;
		bool cmdConfirmFlag = false;
		bool moveCmdFlag = false;
		int moveCmdCnt = 0;

		int CMD_CLIENT_FD_ = 0;
		int DATA_CLIENT_FD_ = 0;

		struct sockaddr_in  cmd_addr_;
		struct sockaddr_in  data_addr_;

		vector<unsigned char> recv_buf_;
	};
}



#endif
