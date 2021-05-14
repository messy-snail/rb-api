#ifndef COMMONHEADER_H
#define COMMONHEADER_H

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

namespace rb {
	const std::string __RB_VERSION__ = "a-0.1";

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
			float   time;                   // time [sec] 0
			float   jnt_ref[6];             // joint reference [deg] 1 - 6
			float   jnt_ang[6];             // joint encoder value [deg] 7 - 12
			float   cur[6];                 // joint current value [mA] 13 - 18
			// 19
			float   tcp_ref[6];             // calculated tool center point from reference [mm, deg] 19 - 24
			float   tcp_pos[6];             // calculated tool center point from encoder [mm, deg] 25 - 30
			// 31
			float   analog_in[4];           // analog input value of control box [V] 31 - 34
			float   analog_out[4];          // analog output value of control box [V] 35 - 38
			int     digital_in[16];         // digital input value of control box [0 or 1] 39 - 54
			int     digital_out[16];        // digital input value of control box [0 or 1] 55 - 70
			// 71
			float   temperature_mc[6];      // board temperature of each joint [celcius] 71 - 76
			// 77
			int     task_pc;                // (ignore) 77
			int     task_repeat;            // (ignore) 78
			int     task_run_id;            // (ignore) 79
			int     task_run_num;           // (ignore) 80
			float   task_run_time;          // (ignore) 81
			int     task_state;             // (ignore) 82
			// 83
			float   default_speed;          // overriding speed [0~1] 83
			int     robot_state;            // state of robot motion [1:idle  2:paused or stopped by accident  3: moving] 84 -> float? int?
			int     power_state;            // power state 85
			// 86
			float   tcp_target[6];          // (ignore) 86 - 91
			int     jnt_info[6];            // joint information (look mSTAT) 92 - 97
			// 98
			int     collision_detect_onoff; // collision detect onoff [0:off  1:on] 98
			int     is_freedrive_mode;      // current freedrive status [0:off  1:on] 99 
			int     program_mode;           // current program mode [0:real mode  1:simulation mode] 100
			// 101
			int     init_state_info;        // status information of robot initialization process 101
			int     init_error;             // error code of robot initialization process 102
			// 103
			float   tfb_analog_in[2];       // analog input value of tool flange board [V] 103 - 104
			int     tfb_digital_in[2];      // digital input value of tool flange board [0 or 1] 105 - 106
			int     tfb_digital_out[2];     // digital output value of tool flange board [0 or 1] 107 - 108
			float   tfb_voltage_out;        // reference voltage of tool flange board [0, 12, 24] 109
			// 110
			int     op_stat_collision_occur; // 110
			int     op_stat_sos_flag; // 111
			int     op_stat_self_collision; // 112
			int     op_stat_soft_estop_occur; // 113
			int     op_stat_ems_flag; //114
			// 115
			int     digital_in_config[2]; //115 - 116
			// 117
			int     inbox_trap_flag[2]; //117 - 118
			int     inbox_check_mode[2]; // 119 - 120
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
	extern systemSTAT   *sys_status;
	extern systemCONFIG *sys_config;
	extern systemPOPUP  *sys_popup;
	extern systemPOPUP  *sys_custom_alarm;
}

#endif // COMMONHEADER_H
