#ifndef COBOT_H
#define COBOT_H

#ifdef RB_EXPORTS
#define COBOT_DECLSPEC __declspec(dllexport)
#else
#define COBOT_DECLSPEC __declspec(dllimport)
#endif

#ifdef RB_EXPORTS
#define POINT_DECLSPEC __declspec(dllexport)
#else
#define POINT_DECLSPEC __declspec(dllimport)
#endif

#ifdef RB_EXPORTS
#define JOINT_DECLSPEC __declspec(dllexport)
#else
#define JOINT_DECLSPEC __declspec(dllimport)
#endif

#include <string>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define MAX_MC					1
#define RT_TIMER_PERIOD_MS		2
#define COMMAND_CANID           0x01

#define MAX_SHARED_DATA			128
#define MAX_CONFIG_DATA			157

// Utility ----------------------------
#define LOG_FAIL    "\033[1;31m"
#define LOG_GOOD    "\033[1;32m"
#define LOG_WARN    "\033[1;33m"
#define LOG_NORMAL  "\033[0m"


//#include "rb-common-header.h"
//#include "rb-point.h"
//#include "rb-joint.h"

using namespace std;

#include <thread>
#include <WinSock2.h>

#pragma comment(lib, "ws2_32")

namespace rb {

	const string __RB_VERSION__ = "a-0.1";

	const int CMD_PORT = 5000;
	const int DATA_PORT = 5001;

	typedef union {
		struct {
			unsigned    FET : 1;	 	// FET ON   //
			unsigned    RUN : 1;		// Control ON
			unsigned    INIT : 1;     // Init Process Passed  //
			unsigned    MOD : 1;		// Control Mode
			unsigned    FRC : 1;		// Friction Compensation //
			unsigned    BAT : 1;      // Low Battery //
			unsigned    CALIB : 1;    // Calibration Mode //
			unsigned    MT_ERR : 1;   // Multi-Turn Error //

			unsigned    JAM : 1;		// JAM Error
			unsigned    CUR : 1;		// Over Current Error
			unsigned    BIG : 1;		// Big Position Error
			unsigned    INP : 1;      // Big Input Error
			unsigned    FLT : 1;		// FET Driver Fault Error //
			unsigned    TMP : 1;      // Temperature Error //
			unsigned    PS1 : 1;		// Position Limit Error (Lower) ////
			unsigned    PS2 : 1;		// Position Limit Error (Upper)

			unsigned    rsvd : 8;

			unsigned    CAN : 1;
			unsigned    rsvd2 : 7;
		}b;
		unsigned char B[4];
	}mSTAT;


	typedef union {
		struct {
			unsigned    ALIVE : 1;
			unsigned    AUTO_MODE : 1;
			unsigned    READY : 1;
			unsigned    READY2 : 1;
			unsigned    ERROR_RESET : 1;
			unsigned    CALIB_START : 1;
			unsigned    CALIB_END : 1;
			unsigned    ALIGN_START : 1;

			unsigned    ALIGN_END : 1;
			unsigned    reserved_1 : 4;
			unsigned    MOVE_CMD : 1;
			unsigned    reserved_2 : 2;

			unsigned    reserved_3 : 8;
			unsigned    reserved_4 : 8;
		}event;
		unsigned char   C[4];
		unsigned int    I;
	}EVENT_VISION;

	typedef union {
		struct {
			unsigned    ALIVE : 1;
			unsigned    AUTO_MODE : 1;
			unsigned    READY : 1;
			unsigned    READY2 : 1;
			unsigned    ERROR_RESET : 1;
			unsigned    CALIB_START : 1;
			unsigned    CALIB_END : 1;
			unsigned    ALIGN_START : 1;

			unsigned    ALIGN_END : 1;
			unsigned    reserved_1 : 4;
			unsigned    MOVE_DONE_ACK : 1;
			unsigned    reserved_2 : 2;

			unsigned    reserved_3 : 8;
			unsigned    reserved_4 : 8;
		}event;
		unsigned char   C[4];
		unsigned int    I;
	}EVENT_ROBOT;

	enum RCR_INIT_ERR_FLAG {
		INIT_STAT_ERR_CLEAR = 0,
		INIT_STAT_ERR_NO_SMPS,
		INIT_STAT_ERR_NO_EMG_SW,
		INIT_STAT_ERR_GPIF_PSW,
		INIT_STAT_ERR_GPIF_FET,
		INIT_STAT_ERR_MC_CAN_CHECK,
		INIT_STAT_ERR_MC_FIND_HOME,
		INIT_STAT_ERR_COL_OFFSET,
	};

	enum RCR_INIT_INFO {
		INIT_STAT_INFO_NOACT = 0,
		INIT_STAT_INFO_VOLTAGE_CHECK,
		INIT_STAT_INFO_DEVICE_CHECK,
		INIT_STAT_INFO_FIND_HOME,
		INIT_STAT_INFO_VARIABLE_CHECK,
		INIT_STAT_INFO_COLLISION_ON,
		INIT_STAT_INFO_INIT_DONE
	};

	typedef struct {
		char    shared_data_name[64];
		int     shared_data_type;
	}shared_data_info;

	typedef union {
		struct {
			char    header[4];
			// 0
			float   time;                   // time [sec]
			float   jnt_ref[6];             // joint reference [deg]
			float   jnt_ang[6];             // joint encoder value [deg]
			float   cur[6];                 // joint current value [mA]
			// 19
			float   tcp_ref[6];             // calculated tool center point from reference [mm, deg]
			float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg]
			// 31
			float   analog_in[4];           // analog input value of control box [V]
			float   analog_out[4];          // analog output value of control box [V]
			int     digital_in[16];         // digital input value of control box [0 or 1]
			int     digital_out[16];        // digital input value of control box [0 or 1]
			// 71
			float   temperature_mc[6];      // board temperature of each joint [celcius]
			// 77
			int     task_pc;                // (ignore)
			int     task_repeat;            // (ignore)
			int     task_run_id;            // (ignore)
			int     task_run_num;           // (ignore)
			float   task_run_time;          // (ignore)
			int     task_state;             // (ignore)
			// 83
			float   default_speed;          // overriding speed [0~1]
			int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving]
			int     power_state;            // power state
			// 86
			float   tcp_target[6];          // (ignore)
			int     jnt_info[6];            // joint information (look mSTAT)
			// 98
			int     collision_detect_onoff; // collision detect onoff [0:off  1:on]
			int     is_freedrive_mode;      // current freedrive status [0:off  1:on]
			int     program_mode;           // current program mode [0:real mode  1:simulation mode]
			// 101
			int     init_state_info;        // status information of robot initialization process
			int     init_error;             // error code of robot initialization process
			// 103
			float   tfb_analog_in[2];       // analog input value of tool flange board [V]
			int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1]
			int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1]
			float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24]
			// 110
			int     op_stat_collision_occur;
			int     op_stat_sos_flag;
			int     op_stat_self_collision;
			int     op_stat_soft_estop_occur;
			int     op_stat_ems_flag;
			// 115
			int     digital_in_config[2];
			// 117
			int     inbox_trap_flag[2];
			int     inbox_check_mode[2];
			// 121
			float eft_fx;
			float eft_fy;
			float eft_fz;
			float eft_mx;
			float eft_my;
			float eft_mz;
		}sdata;
		float fdata[MAX_SHARED_DATA];
		int idata[MAX_SHARED_DATA];
	}systemSTAT;

	typedef union {
		struct {
			char    header[4];
			// 0
			float   sensitivity;            // collision threshold [0~1]
			float   work_x_min;             // (ignore)
			float   work_x_max;             // (ignore)
			float   work_y_min;             // (ignore)
			float   work_y_max;             // (ignore)
			float   work_z_min;             // (ignore)
			float   work_z_max;             // (ignore)
			int     work_onoff;
			float   mount_rotate[3];        // direction of gravity [normalized vector]
			// 11
			float   toolbox_size[3];        // virtual collision box size of tool [mm]
			float   toolbox_center_pos[3];  // virtual collision box position of tool [mm]
			float   tool_mass;              // tool mass [kg]
			float   tool_mass_center_pos[3];// center of mass position of tool [mm]
			float   tool_ee_pos[3];         // tool position [mm]
			// 24
			int     usb_detected_flag;
			int     usb_copy_done_flag;
			// 26
			int     rs485_tool_baud;
			int     rs485_tool_stopbit;
			int     rs485_tool_paritybit;
			int     rs485_box_baud;
			int     rs485_box_stopbit;
			int     rs485_box_paritybit;
			// 32
			int     io_function_in[16];
			int     io_function_out[16];
			// 64
			int     ip_addr[4];
			int     netmask[4];
			int     gateway[4];
			// 76
			int     version;
			// 77
			char    default_script[64];    // 16*4
			// 93
			int     auto_init;
			float   inbox0_size[3];
			float   inbox0_pos[3];
			float   inbox1_size[3];
			float   inbox1_pos[3];
			// 106
			int     default_repeat_num;
			// 107
			float   direct_teaching_sensitivity[6];
			// 113
			float   tool_ee_ori[3];
			// 116
			float   user_coord_0[6];
			float   user_coord_1[6];
			float   user_coord_2[6];
			// 134
			char    dio_begin_box_dout[16]; // 4*4
			float   dio_begin_box_aout[4];
			char    dio_end_box_dout[16];   // 4*4
			float   dio_end_box_aout[4];
			int     dio_begin_tool_voltage; // 1
			int     dio_end_tool_voltage;   // 1
			char    dio_begin_tool_dout[2]; // 1
			char    dio_end_tool_dout[2];   // -
			// 153
			int     ext_ft_model_info;
			// 154
			int     robot_model_type;
			// 155
			int     collision_stop_mode;
			// 156

		}sdata;
		float fdata[MAX_CONFIG_DATA];
	}systemCONFIG;



	typedef struct {
		char    header[4];
		char    type;
		char    msg[1000];
		int     len;
	}systemPOPUP;

	extern int      __IS_PODO_WORKING__;

	extern int      _NO_OF_MC;
	extern systemSTAT* sys_status;
	extern systemCONFIG* sys_config;
	extern systemPOPUP* sys_popup;
	extern systemPOPUP* sys_custom_alarm;
	
	//프로그램 모드
	enum class PG_MODE {
		SIMULATION,
		REAL
	};

	//원동작 타입
	enum class CIRCLE_TYPE {
		INTENDED,
		CONSTANT,
		RADIAL,
		SMOOTH
	};

	//원동작 기준축
	enum class CIRCLE_AXIS {
		X,
		Y,
		Z
	};

	//블렌드 옵션
	enum class BLEND_OPTION {
		RATIO,
		DISTANCE
	};

	//블렌드 회전 옵션
	enum class BLEND_RTYPE {
		INTENDED,
		CONSTANT
	};

	//ITPL 회전 옵션
	enum class ITPL_RTYPE {
		INTENDED,
		CONSTANT,
		RESERVED1,
		SMOOTH,
		RESERVED2,
		CA_INTENDED,
		CA_CONSTANT,
		RESERVED3,
		CA_SMOOTH
	};

	//디지털 출력 옵션
	enum class DOUT_SET {
		LOW,
		HIGH,
		BYPASS
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

	////제어박스 디지털 출력 신호
	//const int SIGNAL_BYPASS = -1;
	//const int SIGNAL_LOW = 0;
	//const int SIGNAL_HIGH = 1;

	////제어박스 디지털 출력 포트
	//using DOUT_PORT = struct dout_port{
	//	int d0 = SIGNAL_BYPASS;		int d1 = SIGNAL_BYPASS; 		int d2 = SIGNAL_BYPASS; 		int d3 = SIGNAL_BYPASS;
	//	int d4 = SIGNAL_BYPASS;		int d5 = SIGNAL_BYPASS; 		int d6 = SIGNAL_BYPASS; 		int d7 = SIGNAL_BYPASS;
	//	int d8 = SIGNAL_BYPASS;		int d9 = SIGNAL_BYPASS; 		int d10 = SIGNAL_BYPASS; 		int d11 = SIGNAL_BYPASS;
	//	int d12 = SIGNAL_BYPASS;		int d13 = SIGNAL_BYPASS; 		int d14 = SIGNAL_BYPASS; 		int d15 = SIGNAL_BYPASS;
	//};

	class POINT_DECLSPEC Point
	{

	private:
		float x_ = 0.f;
		float y_ = 0.f;
		float z_ = 0.f;
		float rx_ = 0.f;
		float ry_ = 0.f;
		float rz_ = 0.f;

		float j0_ = 0.f;
		float j1_ = 0.f;
		float j2_ = 0.f;
		float j3_ = 0.f;
		float j4_ = 0.f;
		float j5_ = 0.f;

	public:
		Point() {}
		Point(const float x, const float y, const float z, const float rx, const float ry, const float rz);
		//Point operator + (Point& p);
		void operator () (const Point p);
		void operator () (const float x, const float y, const float z, const float rx, const float ry, const float rz);
		friend ostream& operator << (ostream& os, rb::Point p) {
			os << "pnt:" << '[' << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << ']';
			return os;
		}


		inline float X() { return x_; }
		inline float Y() { return y_; }
		inline float Z() { return z_; }
		inline float RX() { return rx_; }
		inline float RY() { return ry_; }
		inline float RZ() { return rz_; }


		virtual ~Point();


	};

	class JOINT_DECLSPEC Joint
	{
	private:
		float j0_ = 0.f;
		float j1_ = 0.f;
		float j2_ = 0.f;
		float j3_ = 0.f;
		float j4_ = 0.f;
		float j5_ = 0.f;

		float x_ = 0.f;
		float y_ = 0.f;
		float z_ = 0.f;
		float rx_ = 0.f;
		float ry_ = 0.f;
		float rz_ = 0.f;
	public:
		Joint() {}
		Joint(const float j0, const float j1, const float j2, const float j3, const float j4, const float j5);
		Joint operator + (const Joint& j);
		Joint operator - (const Joint& j);
		friend ostream& operator << (ostream& os, rb::Joint j) {
			os << "jnt:" << '[' << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5() << ']';
			return os;
		}

		virtual ~Joint();

		inline float J0() { return j0_; }
		inline float J1() { return j1_; }
		inline float J2() { return j2_; }
		inline float J3() { return j3_; }
		inline float J4() { return j4_; }
		inline float J5() { return j5_; }
	};

	class COBOT_DECLSPEC Cobot
	{
	public:
		Cobot();
		virtual ~Cobot();

		//제어박스와 통신 연결
		bool ConnectToCB(string ip="10.0.2.7");
		//bool ConnectToCMD(string ip = "10.0.2.7");
		//bool ConnectToData(string ip = "10.0.2.7");

		//제어박스와 통신 끊기
		bool DisConnectToCB();

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

		//Move JL
		bool MoveJL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc);
		bool MoveJL(Point p, float spd, float acc);

		//Move Joint Blend
		bool MoveJB_Clear();
		bool MoveJB_Add(Joint j);
		bool MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5);
		bool MoveJB_Run(float spd, float acc);

		//Move Linear Blend
		bool MovePB_Clear();
		bool MovePB_Add(Point p, float spd, BLEND_OPTION option, float quantity);
		bool MovePB_Add(float x, float y, float z, float rx, float ry, float rz, float spd, BLEND_OPTION option, float quantity);
		bool MovePB_Run(float acc, BLEND_RTYPE type);

		//Move Interpolation
		bool MoveITPL_Clear();
		bool MoveITPL_Add(Point p, float spd);
		bool MoveITPL_Add(float x, float y, float z, float rx, float ry, float rz, float spd);
		bool MoveITPL_Run(float acc, ITPL_RTYPE type);

		//원 동작
		bool MoveCircle_ThreePoint(float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc, CIRCLE_TYPE type);
		bool MoveCircle_ThreePoint(Point p1, Point p2, float spd, float acc, CIRCLE_TYPE type);
		bool MoveCircle_Axis(float x1, float y1, float z1, float rx1, float ry1, float rz1, CIRCLE_AXIS axis, float angle, float spd, float acc, CIRCLE_TYPE type);
		bool MoveCircle_Axis(Point p1, CIRCLE_AXIS axis, float angle, float spd, float acc, CIRCLE_TYPE type);		

		//제어박스 디지털 출력
		//bool ControlBoxDigitalOut(DOUT_PORT port);
		//bool ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15);

		bool CBDigitalOut(float port, DOUT_SET type);
		bool CBAnalogOut(float port, float volt);
		
		//제어박스 아날로그 출력
		//bool ControlBoxAnalogOut(float a0, float a1, float a2, float a3);

		//툴플렌지 출력
		//bool ToolDigitalOut(int d0, int d1, DOUT_VOLT volt);

		//로봇팔 전원 차단
		bool RobotPowerDown(void);

		//default: 30ms
		void SetWaitTime(int millisec);

		//협동로봇 idle 유무
		bool IsIdle();

		//협동로봇 일시정지 유무
		bool IsPause();

		//협동로봇 활성화 유무
		bool IsInitialized();

		//협동로봇 동작 모드
		bool IsRobotReal();

		//커맨드 소켓 연결 확인
		bool IsCommandSockConnect();

		//데이터 소켓 연결 확인
		bool IsDataSockConnect();

		//스크립트 직접 작성
		bool ManualScript(string ex_msg);

		//협동로봇 현재 상태
		COBOT_STATUS GetCurrentCobotStatus();
		
	private:
		void ReadCmd();
		void ReadData();
		void RunThread();
		void ReqDataStart();

		bool isValidIP(string ip);		

		bool socketCmdCom(string ip);
		bool socketCmdClose();

		bool cmd_conneted = false;
		bool data_conneted = false;
		
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
