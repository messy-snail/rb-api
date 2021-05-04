#include "pch.h"
#include "rb-cobot.h"
#include <Ws2tcpip.h>
#include <iomanip>
using namespace rb;

#define PACKET_SIZE 4000


Cobot::Cobot() {
	memset(&systemStat, 0, sizeof(systemSTAT));
	systemStat.sdata.program_mode = -1;
	systemStat.sdata.robot_state = -1;
	
	//rb-api
	cout << "                                    " << endl;
	cout << "      ,--.                     ,--. " << endl;
	cout << ",--.--|  |-.,-----.,--,--.,---.`--' " << endl;
	cout << "|  .--| .-. '-----' ,-.  | .-. ,--. " << endl;
	cout << "|  |  | `-' |     \ '-'  | '-' |  | " << endl;
	cout << "`--'   `---'       `--`--|  |-'`--' " << endl;
	cout << "                         `--'       " << endl;

}

Cobot::~Cobot() {
	socketCmdClose();
	socketDataClose();	
}

bool Cobot::ConnectToCB(string ip) {
	if (isValidIP(ip)) {
		//deep copy
		ip_address_ = ip;
		cout << ip_address_ << endl;
		bool cmd_success = socketCmdCom(ip_address_);
		bool data_success = socketDataCom(ip_address_);
		if (cmd_success&& data_success) {
			RunThread();
			return true;
		}
		else {
			return false;
		}
		//return socketCmdCom(ip_address_) && socketDataCom(ip_address_);
	}
	return false;
}

string Cobot::__Version() {
	cout << "RB-API: "<<__RB_VERSION__ << endl;
	return __RB_VERSION__;
}

Joint Cobot::GetCurrentJoint() {
	current_joint_ = Joint(systemStat.sdata.jnt_ref[0], systemStat.sdata.jnt_ref[1], systemStat.sdata.jnt_ref[2], systemStat.sdata.jnt_ref[3], systemStat.sdata.jnt_ref[4], systemStat.sdata.jnt_ref[5]);
	return current_joint_;
}

tuple<float, float, float, float, float, float> Cobot::GetCurrentSplitedJoint() {
	return make_tuple(systemStat.sdata.jnt_ref[0], systemStat.sdata.jnt_ref[1], systemStat.sdata.jnt_ref[2], systemStat.sdata.jnt_ref[3], systemStat.sdata.jnt_ref[4], systemStat.sdata.jnt_ref[5]);
}

Point Cobot::GetCurrentTCP() {
	current_tcp_ = Point(systemStat.sdata.tcp_ref[0], systemStat.sdata.tcp_ref[1], systemStat.sdata.tcp_ref[2], systemStat.sdata.tcp_ref[3], systemStat.sdata.tcp_ref[4], systemStat.sdata.tcp_ref[5]);
	return current_tcp_;
}

tuple<Joint, Point> Cobot::GetCurrentJP() {
	current_joint_ = Joint(systemStat.sdata.jnt_ref[0], systemStat.sdata.jnt_ref[1], systemStat.sdata.jnt_ref[2], systemStat.sdata.jnt_ref[3], systemStat.sdata.jnt_ref[4], systemStat.sdata.jnt_ref[5]);
	current_tcp_ = Point(systemStat.sdata.tcp_ref[0], systemStat.sdata.tcp_ref[1], systemStat.sdata.tcp_ref[2], systemStat.sdata.tcp_ref[3], systemStat.sdata.tcp_ref[4], systemStat.sdata.tcp_ref[5]);
	return make_tuple(current_joint_, current_tcp_);
}

tuple<float, float, float, float, float, float> Cobot::GetCurrentSplitedTCP() {
	return make_tuple(systemStat.sdata.tcp_ref[0], systemStat.sdata.tcp_ref[1], systemStat.sdata.tcp_ref[2], systemStat.sdata.tcp_ref[3], systemStat.sdata.tcp_ref[4], systemStat.sdata.tcp_ref[5]);
}

bool Cobot::CobotInit() {
	string msg = "mc jall init";
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}

bool Cobot::SetProgramMode(PG_MODE mode) {
	if (mode == PG_MODE::SIMULATION) {
		string msg = "pgmode simulation";
		cmdConfirmFlag = false;
		if (!WSAGetLastError()) {
			send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
			cout << msg << endl;
			return true;
		}
		else {
			return false;
		}
	}
	else if (mode == PG_MODE::REAL) {
		string msg = "pgmode real";
		cmdConfirmFlag = false;
		if (!WSAGetLastError()) {
			send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
			cout << msg << endl;
			return true;
		}
		else {
			return false;
		}
	}
}

bool Cobot::MoveL(Point p, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "movetcp " << spd << ", " << acc << ", " << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ();
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			if (IsIdle()) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run	
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(processing_period_/2);
				cout << ".";
			}
		}		
	}
	else {
		return false;
	}
	
}

bool Cobot::MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "movetcp " << spd << ", " << acc << ", " << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz;
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			if (IsIdle()) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
	}
	else {
		return false;
	}
}


bool Cobot::MoveJ(Joint j, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "jointall " << spd << ", " << acc << ", " << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5()<<" ";
	string msg = oss.str();
	if (!WSAGetLastError() ) {
		while (true) {
			Sleep(wait_time_);
			//cout << "cmdConfirmFlag_MoveJ: " << boolalpha << cmdConfirmFlag << endl;
			//cout << "systemStat.sdata.robot_state_MoveJ: " << systemStat.sdata.robot_state << endl;
			if (IsIdle() && !bReadCmd) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
		//send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		//cout << msg << endl;
		//moveCmdFlag = true;
		//cmdConfirmFlag = false;
		//systemStat.sdata.robot_state = 3; //run
		
		return true;
	}
	else {
		return false;
	}
	
}

bool Cobot::MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "jointall " << spd << ", " << acc << ", " << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5<<" ";
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			if (IsIdle() && !bReadCmd) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
	}
	else {
		return false;
	}
	
}

bool Cobot::MoveJB_Clear() {
	string msg = "blend_jnt clear_pt";
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "blend_jnt add_pt " << spd << ", " << acc << ", " << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5;
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
	//systemStat.sdata.robot_state = 3; //run
}

bool Cobot::MoveJB_Add(Joint j, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "blend_jnt add_pt " << spd << ", " << acc << ", " << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5();
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
	//systemStat.sdata.robot_state = 3; //run
}

bool Cobot::MoveJB_Run() {
	string msg = "blend_jnt move_pt";
	
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			if (IsIdle()) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run	
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
	}
	else {
		return false;
	}
}


bool Cobot::MoveLB_Clear() {
	string msg = "blend_tcp clear_pt";
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}

bool Cobot::MoveLB_Add(Point p, float spd, float acc, float radius) {

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "blend_tcp add_pt " << spd << ", " << acc << ", "<<radius<<", " << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ();
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
	//systemStat.sdata.robot_state = 3; //run

}
bool Cobot::MoveLB_Add(float x, float y, float z, float rx, float ry, float rz, float spd, float acc, float radius) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "blend_tcp add_pt " << spd << ", " << acc << ", " << radius << ", " << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz;
	string msg = oss.str();
	
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
	//systemStat.sdata.robot_state = 3; //run
}

bool Cobot::MoveLB_Run() {
	string msg = "blend_tcp move_pt";
	
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			if (IsIdle()) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				systemStat.sdata.robot_state = 3; //run	
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
	}
	else {
		return false;
	}
	
}

bool Cobot::MoveCircle_ThreePoint(float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc, CIRCLE_TYPE type) {
	string option;
	if (type == CIRCLE_TYPE::INTENDED) {
		option = "intended";
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		option = "constant";
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		option = "radial";
	}
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "movecircle absolute threepoints " << option <<" "<<spd << ", " << acc << ", " << x1 << ", " << y1 << ", " << z1 << ", " << rx1 << ", " << ry1 << ", " << rz1
		<< ", " << x2 << ", " << y2 << ", " << z2 << ", " << rx2 << ", " << ry2 << ", " << rz2;

	string msg = oss.str();
	
	if (!WSAGetLastError() ) {
		while (true) {
			Sleep(wait_time_);
			//cout << "cmdConfirmFlag_MoveJ: " << boolalpha << cmdConfirmFlag << endl;
			//cout << "systemStat.sdata.robot_state_MoveJ: " << systemStat.sdata.robot_state << endl;
			if (IsIdle() && !bReadCmd) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}	
	}
	else {
		return false;
	}
	
}
bool Cobot::MoveCircle_ThreePoint(Point p1, Point p2, float spd, float acc, CIRCLE_TYPE type) {
	string option;
	if (type == CIRCLE_TYPE::INTENDED) {
		option = "intended";
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		option = "constant";
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		option = "radial";
	}
	std::stringstream oss;
	oss << fixed<< setprecision(3);
	oss << "movecircle absolute threepoints " << option << " " << spd << ", " << acc << ", " << p1.X() << ", " << p1.Y() << ", " << p1.Z() << ", " << p1.RX() << ", " << p1.RY() << ", " << p1.RZ()
		<< ", " << p2.X() << ", " << p2.Y() << ", " << p2.Z() << ", " << p2.RX() << ", " << p2.RY() << ", " << p2.RZ();

	string msg = oss.str();
	if (!WSAGetLastError()) {
		while (true) {
			Sleep(wait_time_);
			//cout << "cmdConfirmFlag_MoveJ: " << boolalpha << cmdConfirmFlag << endl;
			//cout << "systemStat.sdata.robot_state_MoveJ: " << systemStat.sdata.robot_state << endl;
			if (IsIdle() && !bReadCmd) {
				send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
				cout << msg << endl;
				moveCmdFlag = true;
				cmdConfirmFlag = false;
				systemStat.sdata.robot_state = 3; //run
				return true;
			}
			else if (IsPause()) {
				cout << "paused" << endl;
				return false;
			}
			else {
				//Sleep(WAIT_TIME);
				cout << ".";
			}
		}
	}
	else {
		return false;
	}
	
}


//void Cobot::MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd, float acc) {
//	QString text;
//	char buf[15];
//	if (type == 0) {
//		sprintf(buf, "intended");
//	}
//	else if (type == 1) {
//		sprintf(buf, "constant");
//	}
//	else if (type == 2) {
//		sprintf(buf, "radial");
//	}
//	text.sprintf("movecircle absolute axis %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
//		buf, spd, acc, rot_angle, cx, cy, cz, ax, ay, az);
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3;
//}

bool Cobot::ControlBoxDigitalOut(DOUT_PORT port) {
	std::stringstream oss;

	oss << "digital_out " << static_cast<int>(port.d0) << ", " << static_cast<int>(port.d1) << ", " << static_cast<int>(port.d2) << ", " << static_cast<int>(port.d3) << ", " << static_cast<int>(port.d4) << ", "
		<< static_cast<int>(port.d5) << ", " << static_cast<int>(port.d6) << ", " << static_cast<int>(port.d7) << ", " << static_cast<int>(port.d8)	<< ", " << static_cast<int>(port.d9) << ", " << static_cast<int>(port.d10) << ", "
		<< static_cast<int>(port.d11) << ", " << static_cast<int>(port.d12) << ", " << static_cast<int>(port.d13) << ", " << static_cast<int>(port.d14) << ", " << static_cast<int>(port.d15);

	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15) {
	std::stringstream oss;
	oss << "digital_out " << d0 << ", " << d1 << ", " << d2 << ", " << d3 << ", " << d4 << ", " << d5 << ", " << d6 << ", " << d7 << ", " << d8
		<< ", " << d9 << ", " << d10 << ", " << d11 << ", " << d12 << ", " << d13 << ", " << d14<<", "<<d15;

	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
	
}
bool Cobot::ControlBoxAnalogOut(float a0, float a1, float a2, float a3) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "analog_out " << a0 << ", " << a1 << ", " << a2 << ", " << a3;

	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}

//bool Cobot::TestPlay(){
//	string msg = "task play";
//	send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
//	return true;
//}
//bool Cobot::TestStop() {
//	string msg = "task stop";
//	send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
//	return true;
//}

//volt 12, 24
bool Cobot::ToolOut(int d0, int d1, DOUT_VOLT volt) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	int volt_val;
	if (volt == DOUT_VOLT::VOLT_12) {
		volt_val = 12;
	}
	else if (volt == DOUT_VOLT::VOLT_24) {
		volt_val = 24;
	}
	else if (volt == DOUT_VOLT::VOLT_0) {
		volt_val = 0;
	}
	else {
		volt_val = 0;
	}
	
	oss << "tool_out " << volt_val << ", " << d0 << ", " << d1;

	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}

bool Cobot::SetBaseSpeed(float spd) {
	
	if (spd > 1.0) {
		spd = 1.0;
	}
	else if (spd < 0.0) {
		spd = 0.0;
	}

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "sdw default_speed " << spd;
	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::MotionPause() {
	string msg = "task pause";
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}

bool Cobot::MotionHalt() {
	string msg = "task stop";
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::MotionResume() {
	string msg = "task resume_a";
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::CollisionResume() {
	string msg = "task resume_b";
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}
bool Cobot::SetMotionBreak(string condition, float dec_time) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "set_motion_break " << condition<<","<< dec_time;
	string msg = oss.str();

	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
		cmdConfirmFlag = false;
		return true;
	}
	else {
		return false;
	}
}


//void Cobot::ReadCmd() {
//	char buffer[PACKET_SIZE] = {}; //char 생성
//	string cmd; //string 생성
//	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;
//	while (!WSAGetLastError()) {
//		ZeroMemory(&buffer, PACKET_SIZE); //buffer 비우기
//		//ioctlsocket()
//		recv(CMD_CLIENT_FD_, buffer, PACKET_SIZE, 0); //데이터받아오기
//		cmd = buffer; //buffer의값이 cmd에 들어갑니다
//		cout << "[cmd]" << cmd << endl;
//		if (cmd.compare("The command was executed\n") == 0) {
//			cmdConfirmFlag = true;
//			if (moveCmdFlag == true) {
//				moveCmdCnt = 4;
//				systemStat.sdata.robot_state = 3;
//				moveCmdFlag = false;
//			}
//			cout << "moveCmdFlag: " << boolalpha << moveCmdFlag << endl;
//		}
//	}	
//	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;	
//}

void Cobot::ReadCmd() {
	
	while (true) {
		Sleep(10);
		std::vector<char> buffer(PACKET_SIZE);
		int result = recv(CMD_CLIENT_FD_, buffer.data(), buffer.size(), 0);
		if (result != -1) {
			buffer.resize(result);
		}
		else {
			// Handle error
		}
		//string 생성
		string cmd(buffer.begin(), buffer.end());

		//cout << "[cmd]" << cmd << endl;
		if (cmd.compare("The command was executed\n") == 0) {
			bReadCmd = true;
			//cout << "[cmd2]" << cmd << endl;
			cmdConfirmFlag = true;
			//cout << "cmdConfirmFlag1: " << boolalpha << cmdConfirmFlag << endl;
			//cout << "moveCmdFlag1: " << boolalpha << moveCmdFlag << endl;
			//cout << "systemStat.sdata.robot_state1: " << systemStat.sdata.robot_state << endl;
			if (moveCmdFlag == true) {
				moveCmdCnt = 3;
				systemStat.sdata.robot_state = 3;				
				moveCmdFlag = false;
			}
			else {
				//return;
			}
			bReadCmd = false;
		}
		//cout << "cmdConfirmFlag2: " << boolalpha << cmdConfirmFlag << endl;
		//cout << "moveCmdFlag2: " << boolalpha << moveCmdFlag << endl;
		//cout << "systemStat.sdata.robot_state2: " << systemStat.sdata.robot_state << endl;
		if (systemStat.sdata.robot_state == 1) {
			int a = 0;
		}
		buffer.clear();
	}
	
}


void Cobot::ReqDataStart() {
	while (true) {
		string msg = "reqdata";
		if (!WSAGetLastError()) {
			send(DATA_CLIENT_FD_, msg.c_str(), msg.length(), 0);
			//cout << msg << endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

	}
}

void Cobot::RunThread() {
	thread proc1 = thread(&rb::Cobot::ReadCmd, this);
	thread proc2 = thread(&rb::Cobot::ReqDataStart, this);
	thread proc3 = thread(&rb::Cobot::ReadData, this);
	proc1.detach();
	proc2.detach();
	proc3.detach();
}

void Cobot::SetWaitTime(int millisec) {
	wait_time_ = millisec;
	cout << wait_time_<<"ms" << endl;
}

//void Cobot::JoinThread() {
//	proc1.join();
//	proc2.join();
//	proc3.join();
//}
void Cobot::ReadData() {
	std::vector<char> buffer(PACKET_SIZE);

	while (!WSAGetLastError()) {
		int result = recv(DATA_CLIENT_FD_, buffer.data(), buffer.size(), 0);
		if (result != -1) {
			buffer.resize(result);
			recv_buf_.insert(recv_buf_.end(), buffer.begin(), buffer.end());
		}
		else {
			// Handle error
		}
		
		int tempSize;
		while (recv_buf_.size() > 4) {
			if (recv_buf_[0] == '$') {
				int size = (int(unsigned char(recv_buf_[2]) << 8) | int(unsigned char(recv_buf_[1])));
				if (size <= recv_buf_.size() - 4)
				{
					//int templen = recv_buf_.size();
					if (3 == recv_buf_[3]) {
						if (moveCmdCnt > 0) {
							moveCmdCnt--;
						}
						else {
							tempSize = sizeof(systemSTAT) <= size ? sizeof(systemSTAT) : size;
							memcpy(&systemStat, recv_buf_.data(), tempSize);
						}
						for (int i = 0; i < size + 4; i++) {
							recv_buf_.erase(recv_buf_.begin());
						}
					}
					else if (4 == recv_buf_[3]) {
						tempSize = sizeof(systemCONFIG) <= size ? sizeof(systemCONFIG) : size;
						memcpy(&systemConfig, recv_buf_.data(), tempSize);
						for (int i = 0; i < size + 4; i++) {
							recv_buf_.erase(recv_buf_.begin());
						}

					}
					else if (10 == recv_buf_[3]) {
						tempSize = sizeof(systemPopup) <= size ? sizeof(systemPopup) : size;
						memcpy(&systemPopup, recv_buf_.data(), tempSize);
						for (int i = 0; i < size + 4; i++) {
							recv_buf_.erase(recv_buf_.begin());
						}
					}
					else {
						recv_buf_.erase(recv_buf_.begin());
					}
				}
			}
		}
		if (systemStat.sdata.robot_state == 2) {
			int a = 0;
		}
		else if (systemStat.sdata.robot_state == 3) {
			int a = 0;
		}
	}
	
}



bool Cobot::isValidIP(string ip) {
	stringstream ss(ip);
	string item;

	int iter = 0;
	while (getline(ss, item, '.')) {
		int number = stoi(item);
		if (number < 0) {
			throw "[Error] Valid range 0~255";
			return false;
		}
		else if (number > 255) {
			throw "[Error] Valid range 0~255";
			return false;
		}
		iter++;
	}

	if (iter == 4) {
		return true;
	}
	else {
		throw "[Error] 4 elements";
		return false;
	}

}

bool Cobot::IsIdle() {
	//return ((cmdConfirmFlag == true) && (systemStat.sdata.robot_state == 1));
	return (systemStat.sdata.robot_state == 1);
}

bool Cobot::IsPause() {
	//return ((cmdConfirmFlag == true) && (systemStat.sdata.robot_state == 1));
	return (systemStat.sdata.op_stat_soft_estop_occur == 1);
}

COBOT_STATUS Cobot::GetCurrentCobotStatus() {

	//PAUSED 유무는 op_stat_soft_estop_occur으로 확인해야함.
	//systemStat.sdata.robot_state로는 확인 불가.
	if (systemStat.sdata.op_stat_soft_estop_occur == 1) {
		return COBOT_STATUS::PAUSED;
	}
	if (systemStat.sdata.robot_state == 1) {
		return COBOT_STATUS::IDLE;
	}	
	else if (systemStat.sdata.robot_state == 3) {
		return COBOT_STATUS::RUNNING;
	}
	else {
		return COBOT_STATUS::UNKNOWN;
	}
}
bool Cobot::socketCmdCom(string ip) {
	WSADATA wsa;
	//2.2 version
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0){
		cout << "Failed. Error code: %d\n\n" << WSAGetLastError() << endl;
		exit(1);
	}
	//data_sock_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	CMD_CLIENT_FD_ = socket(AF_INET, SOCK_STREAM, 0);
	//cout << "Command socket : " << CMD_CLIENT_FD_ << endl;
	if (CMD_CLIENT_FD_ == -1) {
		//exception
	}
	inet_pton(AF_INET, ip.c_str(), &cmd_addr_.sin_addr.s_addr);

	cmd_addr_.sin_family = AF_INET;
	cmd_addr_.sin_port = htons(CMD_PORT);

	int optval = 1;
	const char optval2 = (const char)optval;

	setsockopt(CMD_CLIENT_FD_, SOL_SOCKET, SO_REUSEADDR, &optval2, sizeof(optval));
	//setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	setsockopt(CMD_CLIENT_FD_, IPPROTO_TCP, TCP_NODELAY, &optval2, sizeof(optval));

	if (connect(CMD_CLIENT_FD_, (struct sockaddr*)&cmd_addr_, sizeof(cmd_addr_)) < 0){
		cout << "CMD connect : " << CMD_CLIENT_FD_ << endl;
		/*cout << "Connect2Server return false" << endl;*/
		cout << "Cmd client connect to server..!!  Fail. Check your ip address" << endl;
		return false;
	}
	cout << "Cmd client connect to server..!!  Success" << endl;

	unsigned long arg = 1;
	return true;
	//return ioctlsocket(CMD_CLIENT_FD_, FIONBIO, &arg) == 0;
}


bool Cobot::socketCmdClose() {
	WSACleanup();
	return closesocket(CMD_CLIENT_FD_);
}

bool Cobot::socketDataCom(string ip) {
	WSADATA wsa;
	//2.2 version
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		cout << "Failed. Error code: %d\n\n" << WSAGetLastError() << endl;
		exit(1);
	}
	//data_sock_ = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	DATA_CLIENT_FD_ = socket(AF_INET, SOCK_STREAM, 0);
	//cout << "Data socket : " << DATA_CLIENT_FD_ << endl;
	if (DATA_CLIENT_FD_ == -1) {
		//exception
	}
	inet_pton(AF_INET, ip.c_str(), &data_addr_.sin_addr.s_addr);

	data_addr_.sin_family = AF_INET;
	data_addr_.sin_port = htons(DATA_PORT);

	int optval = 1;
	const char optval2 = (const char)optval;

	setsockopt(DATA_CLIENT_FD_, SOL_SOCKET, SO_REUSEADDR, &optval2, sizeof(optval));
	//setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	setsockopt(DATA_CLIENT_FD_, IPPROTO_TCP, TCP_NODELAY, &optval2, sizeof(optval));

	if (connect(DATA_CLIENT_FD_, (struct sockaddr*)&data_addr_, sizeof(data_addr_)) < 0) {
		cout << "Data connect : " << DATA_CLIENT_FD_ << endl;
		cout << "Data client connect to server..!!  Fail. Check your ip address" << endl;
		return false;
	}
	cout << "Data client connect to server..!!  Success" << endl;

	unsigned long arg = 1;
	return true;
}

bool Cobot::socketDataClose() {
	WSACleanup();
	return closesocket(DATA_CLIENT_FD_);
}

