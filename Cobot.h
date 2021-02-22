#ifndef COBOT_H
#define COBOT_H


#ifdef RB_EXPORTS
#define COBOT_DECLSPEC __declspec(dllexport)
#else
#define COBOT_DECLSPEC __declspec(dllimport)
#endif

#include "CommonHeader.h"
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
		//void ProgramMode_Real();
	private:
		string ip_address_;
		bool isValidIP(string ip);
		bool isMotionIdle();
		bool socketCmdCom(string ip);
		bool socketCmdClose();
		void readyCmdCom();

		systemSTAT systemStat;
		systemCONFIG systemConfig;
		systemPOPUP  systemPopup;

		bool initFlag = false;
		bool cmdConfirmFlag = false;
		bool moveCmdFlag = false;
		int moveCmdCnt = 0;
		int CMD_CLIENT_FD_ = 0;

		struct sockaddr_in  ClientAddr;

	};
	/*extern "C" {
		CONTROL_DECLSPEC void PrintTest();
	}*/
}



#endif
