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

		cmd_conneted = cmd_success;
		data_conneted = data_success;

		if (cmd_success && data_success) {
			RunThread();
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool Cobot::DisConnectToCB() {//data socket의 close의 경우 바로 return이 0이 돌아오지 않는다.
	cout << "disconnect cb" << endl;
	bool cmd = socketCmdClose();
	bool data = socketDataClose();

	Sleep(1000);

	if (1) {
		cmd_conneted = false;
		data_conneted = false;
		return true;
	}		
	else {
		return false;
	}
		
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
	string msg = "mc jall init ";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::SetProgramMode(PG_MODE mode) {
	if (mode == PG_MODE::SIMULATION) {
		string msg = "pgmode simulation ";
		cmdConfirmFlag = false;
		return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
	}
	else if (mode == PG_MODE::REAL) {
		string msg = "pgmode real ";
		cmdConfirmFlag = false;
		return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
	}
}

bool Cobot::MoveL(Point p, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_l(pnt[" << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << "], " << spd << ", " << acc << ") ";

	string msg = oss.str();
	
	return SendCOMMAND(msg, CMD_TYPE::MOVE);	
}

bool Cobot::MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_l(pnt[" << x << ", " << y << ", " << z << ", " << rx<< ", " << ry << ", " << rz << "], " << spd << ", " << acc << ") ";
	string msg = oss.str();
	
	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}


bool Cobot::MoveJ(Joint j, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_j(jnt[" << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5() << "], " << spd << ", " << acc << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_j(jnt[" << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5 << "], " << spd << ", " << acc << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveJL(Point p, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_jl(pnt[" << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << "], " << spd << ", " << acc << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveJL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_jl(pnt[" << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz << "], " << spd << ", " << acc << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveJB_Clear() {
	string msg = "move_jb_clear() ";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}
bool Cobot::MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_jb_add(jnt[" << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5 << "]) ";
	string msg = oss.str();
	
	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MoveJB_Add(Joint j) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_jb_add(jnt[" << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5() << "]) ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MoveJB_Run(float spd, float acc) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_jb_run(" << spd <<", " << acc << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}


bool Cobot::MovePB_Clear() {
	string msg = "move_pb_clear() ";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MovePB_Add(Point p, float spd, BLEND_OPTION option, float quantity) {

	float b_option;
	if (option == BLEND_OPTION::DISTANCE)
	{
		b_option = 1;
		if (quantity < 0)
			quantity = 0;
	}
	else if (option == BLEND_OPTION::RATIO)
	{
		b_option = 0;
		if (quantity > 1)
			quantity = 1;
		else if (quantity < 0)
			quantity = 0;
	}

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_pb_add(pnt[" << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << "], " << spd << ", " << b_option << "," << quantity << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}
bool Cobot::MovePB_Add(float x, float y, float z, float rx, float ry, float rz, float spd, BLEND_OPTION option, float quantity) {

	float b_option;
	if (option == BLEND_OPTION::DISTANCE)
	{
		b_option = 1;
		if (quantity < 0)
			quantity = 0;
	}
	else if (option == BLEND_OPTION::RATIO)
	{
		b_option = 0;
		if (quantity > 1)
			quantity = 1;
		else if (quantity < 0)
			quantity = 0;
	}

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_pb_add(pnt[" << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz << "], " << spd << ", " << b_option << "," << quantity << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MovePB_Run(float acc, BLEND_RTYPE type) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_pb_run(" << acc << "," << (float)type << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveITPL_Clear() {
	string msg = "move_itpl_clear() ";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MoveITPL_Add(Point p, float spd) {

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_itpl_add(pnt[" << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ() << "], " << spd << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MoveITPL_Add(float x, float y, float z, float rx, float ry, float rz, float spd) {

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_itpl_add(pnt[" << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz << "], " << spd << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MoveITPL_Run(float acc, ITPL_RTYPE type) {
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_itpl_run(" << acc << "," << (float)type << ") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveCircle_ThreePoint(float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc, CIRCLE_TYPE type) {
	string option;
	float r_option=0.;
	if (type == CIRCLE_TYPE::INTENDED) {
		r_option = (float)CIRCLE_TYPE::INTENDED;
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		r_option = (float)CIRCLE_TYPE::CONSTANT;
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		r_option = (float)CIRCLE_TYPE::RADIAL;
	}
	else if (type == CIRCLE_TYPE::SMOOTH) {
		r_option = (float)CIRCLE_TYPE::SMOOTH;
	}
	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_c_points(pnt[" << x1 << ", " << y1 << ", " << z1 << ", " << rx1 << ", " << ry1 << ", " << rz1 << "], pnt["
		<< x2 << ", " << y2 << ", " << z2 << ", " << rx2 << ", " << ry2 << ", " << rz2 << "] ," << spd << ", " << acc << ", " << r_option << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveCircle_ThreePoint(Point p1, Point p2, float spd, float acc, CIRCLE_TYPE type) {
	string option;
	float r_option;
	if (type == CIRCLE_TYPE::INTENDED) {
		r_option = (float)CIRCLE_TYPE::INTENDED;
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		r_option = (float)CIRCLE_TYPE::CONSTANT;
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		r_option = (float)CIRCLE_TYPE::RADIAL;
	}
	else if (type == CIRCLE_TYPE::SMOOTH) {
		r_option = (float)CIRCLE_TYPE::SMOOTH;
	}
	std::stringstream oss;
	oss << fixed<< setprecision(3);
	oss << "move_c_points(pnt[" << p1.X() << ", " << p1.Y() << ", " << p1.Z() << ", " << p1.RX() << ", " << p1.RY() << ", " << p1.RZ() << "], pnt["
		<< p2.X() << ", " << p2.Y() << ", " << p2.Z() << ", " << p2.RX() << ", " << p2.RY() << ", " << p2.RZ() << "] ," << spd << ", " << acc << ", " << r_option << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveCircle_Axis(float x1, float y1, float z1, float rx1, float ry1, float rz1, CIRCLE_AXIS axis, float direction, float angle, float spd, float acc, CIRCLE_TYPE type)
{
	float r_option;
	string a_option;
	if (type == CIRCLE_TYPE::INTENDED) {
		r_option = (float)CIRCLE_TYPE::INTENDED;
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		r_option = (float)CIRCLE_TYPE::CONSTANT;
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		r_option = (float)CIRCLE_TYPE::RADIAL;
	}

	if (axis == CIRCLE_AXIS::X) {
		if(direction == 1)
			a_option = "1,0,0";
		else if(direction == -1)
			a_option = "-1,0,0";
		else
			a_option = "1,0,0";
	}
	else if (axis == CIRCLE_AXIS::Y) {
		if (direction == 1)
			a_option = "0,1,0";
		else if (direction == -1)
			a_option = "0,-1,0";
		else
			a_option = "1,0,0";
	}
	else if (axis == CIRCLE_AXIS::Z) {
		if (direction == 1)
			a_option = "0,0,1";
		else if (direction == -1)
			a_option = "0,0,-1";
		else
			a_option = "1,0,0";
	}
	else
		cout << "axis_error";

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_c_axis(pnt[" << x1 << ", " << y1 << ", " << z1 << ", " << rx1 << ", " << ry1 << ", " << rz1 << "], " << a_option
		<< "," << angle << "," << spd << ", " << acc << "," << r_option << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::MoveCircle_Axis(Point p1, CIRCLE_AXIS axis, float direction, float angle, float spd, float acc, CIRCLE_TYPE type)
{
	float r_option;
	string a_option;
	if (type == CIRCLE_TYPE::INTENDED) {
		r_option = (float)CIRCLE_TYPE::INTENDED;
	}
	else if (type == CIRCLE_TYPE::CONSTANT) {
		r_option = (float)CIRCLE_TYPE::CONSTANT;
	}
	else if (type == CIRCLE_TYPE::RADIAL) {
		r_option = (float)CIRCLE_TYPE::RADIAL;
	}

	if (axis == CIRCLE_AXIS::X) {
		if (direction == 1)
			a_option = "1,0,0";
		else if (direction == -1)
			a_option = "-1,0,0";
		else
			a_option = "1,0,0";
	}
	else if (axis == CIRCLE_AXIS::Y) {
		if (direction == 1)
			a_option = "0,1,0";
		else if (direction == -1)
			a_option = "0,-1,0";
		else
			a_option = "1,0,0";
	}
	else if (axis == CIRCLE_AXIS::Z) {
		if (direction == 1)
			a_option = "0,0,1";
		else if (direction == -1)
			a_option = "0,0,-1";
		else
			a_option = "1,0,0";
	}
	else
		cout << "axis_error";

	std::stringstream oss;
	oss << fixed << setprecision(3);
	oss << "move_c_axis(pnt[" << p1.X() << ", " << p1.Y() << ", " << p1.Z() << ", " << p1.RX() << ", " << p1.RY() << ", " << p1.RZ() << "], " << a_option
		<< "," << angle << "," << spd << ", " << acc << "," << r_option << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::MOVE);
}

bool Cobot::CBDigitalOut(float port, DOUT_SET type) {
	float dtype;

	if (type == DOUT_SET::LOW)
		dtype = 0;
	else if (type == DOUT_SET::HIGH)
		dtype = 1;
	else if (type == DOUT_SET::BYPASS)
		dtype = -1;

	if (port > 15 || port < 0)
		return false;

	std::stringstream oss;
	oss << "set_box_dout(" << port << dtype << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::CBAnalogOut(float port, float volt) {
	float dtype;

	if (volt > 10 || volt < 0)
		return false;

	if (port > 3 || port < 0)
		return false;

	std::stringstream oss;
	oss << "set_box_aout(" << port << volt << ") ";

	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
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
	oss << "set_speed_bar(" << spd<<") ";
	string msg = oss.str();

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}
bool Cobot::MotionPause() {
	string msg = "task pause";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::MotionHalt() {
	string msg = "task stop";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}
bool Cobot::MotionResume() {
	string msg = "task resume_a";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}
bool Cobot::CollisionResume() {//when external collision was detected, use this function.
	string msg = "task resume_b";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::RobotPowerDown(void) {
	string msg = "arm_powerdown() ";

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

bool Cobot::ManualScript(string ex_msg) {
	string msg = ex_msg;

	return SendCOMMAND(msg, CMD_TYPE::NONMOVE);
}

void Cobot::ReadCmd() {
	
	while (true) {
		Sleep(10);
		std::vector<char> buffer(PACKET_SIZE);
		int result = recv(CMD_CLIENT_FD_, buffer.data(), buffer.size(), 0);
		if (result != -1) {
			buffer.resize(result);
			//cout << "test" << endl;
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
			//cout << "test" << endl;
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
					cout << size << endl;
					//int templen = recv_buf_.size();
					if (3 == recv_buf_[3]) {
						cout << "stat" << endl;
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
						cout << "config" << endl;
						memcpy(&systemConfig, recv_buf_.data(), tempSize);
						for (int i = 0; i < size + 4; i++) {
							recv_buf_.erase(recv_buf_.begin());
						}

					}
					else if (10 == recv_buf_[3]) {
						cout << "popup" << endl;
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

bool Cobot::IsInitialized() {
	if (systemStat.sdata.init_state_info == 6)
		return true;
	else
		return false;
}

bool Cobot::IsRobotReal() {
	if (systemStat.sdata.program_mode == 0)
		return true;
	else
		return false;
}

bool Cobot::IsCommandSockConnect() {
	
	string status;
	if (cmd_conneted == true) {
		status = "Command Connected";
		cout << status << endl;
		return true;
	}
	else {
		status = "Command DisConnect";
		cout << status << endl;
		return false;
	}	
}

bool Cobot::IsDataSockConnect() {
	string status;
	if (data_conneted == true) {
		status = "Data Connected";
		cout << status << endl;
		return true;
	}
	else {
		status = "Data DisConnect";
		cout << status << endl;
		return false;
	}
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
	//cmd_connected = false;
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
	//data_conneted = false;
	return closesocket(DATA_CLIENT_FD_);
}

bool Cobot::SendCOMMAND(string str, CMD_TYPE cmd_type) {
	if (cmd_type == CMD_TYPE::MOVE)
	{
		if (!WSAGetLastError()) {
			while (true) {
				Sleep(wait_time_);
				if (IsIdle() && !bReadCmd) {
					send(CMD_CLIENT_FD_, str.c_str(), str.length(), 0);
					cout << str << endl;
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
					cout << ".";
				}
			}
		}
		else {
			return false;
		}
	}
	else {
		if (!WSAGetLastError()) {
			send(CMD_CLIENT_FD_, str.c_str(), str.length(), 0);
			cmdConfirmFlag = false;
			cout << str << endl;
			return true;
		}
		else {
			return false;
		}
	}
}