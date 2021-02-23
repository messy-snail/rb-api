#include "pch.h"
#include "Cobot.h"
#include <Ws2tcpip.h>
#include <iomanip>
using namespace rb;

#define PACKET_SIZE 4000



Cobot::Cobot() {
	memset(&systemStat, 0, sizeof(systemSTAT));
	systemStat.sdata.program_mode = -1;
	systemStat.sdata.robot_state = -1;
}

Cobot::~Cobot() {
	socketCmdClose();
	socketDataClose();	
}

//bool Cobot::ConnectToCB(string ip) {
//	bool ret = isValidIP(ip);
//	if (ret) {
//		//deep copy
//		ip_address_ = ip;
//		cout << ip_address_ << endl;
//		if (socketCmdCom(ip_address_)) {
//			proc = thread(&Cobot::readyCmdCom, this);	
//			//SetTimer(NULL, 1, 50, readyCmdCom);
//		}	
//		//char msg[PACKET_SIZE] = { 0 };
//		//while (!WSAGetLastError()) {
//		//	if (bSend_) {
//		//		send(CMD_CLIENT_FD_, sending_msg_.c_str(), sending_msg_.length(), 0);
//		//		bSend_ = false;
//		//	}
//		//}
//		SetProgramMode(PG_MODE::REAL);
//		proc.join();
//		return true;
//	}
//	return false;
//}

bool Cobot::ConnectToCB(string ip) {
	bool ret = isValidIP(ip);
	if (ret) {
		//deep copy
		ip_address_ = ip;
		cout << ip_address_ << endl;
		return socketCmdCom(ip_address_) && socketDataCom(ip_address_);
	}
	return false;
}

string Cobot::Version() {
	cout << __RB_VERSION__ << endl;
	return __RB_VERSION__;
}

const Point Cobot::GetCurrentPoint() {
	//current_point_ = Point(systemStat.sdata.jnt_ang[0], systemStat.sdata.jnt_ang[0], systemStat.sdata.jnt_ang[0], systemStat.sdata.jnt_ang[0], systemStat.sdata.jnt_ang[0], systemStat.sdata.jnt_ang[0]);
	Point p1(5, 5, 5, 5, 5, 5);
	current_point_(p1);
	return current_point_;
}

void Cobot::CobotInit() {
	string msg = "mc jall init";
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
}

void Cobot::SetProgramMode(PG_MODE mode) {
	if (mode == PG_MODE::SIMULATION) {
		string msg = "pgmode simulation";
		cmdConfirmFlag = false;
		if (!WSAGetLastError()) {
			send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
			cout << msg << endl;
		}
	}
	else if (mode == PG_MODE::REAL) {
		string msg = "pgmode real";
		cmdConfirmFlag = false;
		if (!WSAGetLastError()) {
			send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
			cout << msg << endl;
		}
	}
}

void Cobot::MoveL(Point p, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "movetcp " << spd << ", " << acc << ", " << p.X() << ", " << p.Y() << ", " << p.Z() << ", " << p.RX() << ", " << p.RY() << ", " << p.RZ();
	string msg = oss.str();
	moveCmdFlag = true;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run	
}

void Cobot::MoveL(float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "movetcp " << spd << ", " << acc << ", " << x << ", " << y << ", " << z << ", " << rx << ", " << ry << ", " << rz;
	string msg = oss.str();
	moveCmdFlag = true;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run	
}


void Cobot::MoveJ(Joint j, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "jointall " << spd << ", " << acc << ", " << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5();
	string msg = oss.str();
	moveCmdFlag = true;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run
}

void Cobot::MoveJ(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "jointall " << spd << ", " << acc << ", " << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5;
	string msg = oss.str();
	moveCmdFlag = true;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run
}

void Cobot::MoveJB_Clear() {
	string msg = "blend_jnt clear_pt";
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
}
void Cobot::MoveJB_Add(float j0, float j1, float j2, float j3, float j4, float j5, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "blend_jnt add_pt " << spd << ", " << acc << ", " << j0 << ", " << j1 << ", " << j2 << ", " << j3 << ", " << j4 << ", " << j5;
	string msg = oss.str();
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run
}

void Cobot::MoveJB_Add(Joint j, float spd, float acc) {
	std::stringstream oss;
	oss << setprecision(3);
	oss << "blend_jnt add_pt " << spd << ", " << acc << ", " << j.J0() << ", " << j.J1() << ", " << j.J2() << ", " << j.J3() << ", " << j.J4() << ", " << j.J5();
	string msg = oss.str();
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3; //run
}

void Cobot::MoveJB_Run() {
	string msg = "blend_jnt move_pt";
	moveCmdFlag = true;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
		cout << msg << endl;
	}
	systemStat.sdata.robot_state = 3;
}


//void Cobot::MoveTCPBlend_Clear() {
//	QString text;
//	text.sprintf("blend_tcp clear_pt");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
//	QString text;
//	text.sprintf("blend_tcp add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, radius, x, y, z, rx, ry, rz);
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3; //run
//}
//void Cobot::MoveTCPBlend_MovePoint() {
//	QString text;
//	text.sprintf("blend_tcp move_pt");
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3;
//}

//
//void Cobot::MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd, float acc) {
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
//	text.sprintf("movecircle absolute threepoints %s %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
//		buf, spd, acc, x1, y1, z1, rx1, ry1, rz1, x2, y2, z2, rx2, ry2, rz2);
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3;
//}
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

//void Cobot::ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15) {
//	QString text;
//	text.sprintf("digital_out %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", d0, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15);
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::ControlBoxAnalogOut(float a0, float a1, float a2, float a3) {
//	QString text;
//	text.sprintf("analog_out %.3f, %.3f, %.3f, %.3f", a0, a1, a2, a3);
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::ToolOut(int volt, int d0, int d1) {
//	int temp_volt = volt;
//	if ((temp_volt != 12) && (temp_volt != 24))
//		temp_volt = 0;
//
//	QString text;
//	text.sprintf("tool_out %d, %d, %d", temp_volt, d0, d1);
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::BaseSpeedChange(float spd) {
//	QString text;
//	if (spd > 1.0)
//		spd = 1.0;
//	if (spd < 0.0)
//		spd = 0.0;
//	text.sprintf("sdw default_speed %.3f", spd);
//	cout << "speed control" << endl;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::MotionPause() {
//	QString text;
//	text.sprintf("task pause");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::MotionHalt() {
//	QString text;
//	text.sprintf("task stop");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::MotionResume() {
//	QString text;
//	text.sprintf("task resume_a");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::CollisionResume() {
//	QString text;
//	text.sprintf("task resume_b");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::SetMotionBreak(QString condition, float dec_time) {
//	QString text;
//	text.sprintf("set_motion_break %s,%.3f", condition.toStdString().data(), dec_time);
//	qDebug() << text;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}


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
	std::vector<char> buffer(PACKET_SIZE);
	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;
	while (!WSAGetLastError()) {
		int result = recv(CMD_CLIENT_FD_, buffer.data(), buffer.size(), 0);
		if (result != -1) {
			buffer.resize(result);
		}
		else {
			// Handle error
		}
		//string 생성
		string cmd(buffer.begin(), buffer.end());
		
		cout << "[cmd]" << cmd << endl;
		if (cmd.compare("The command was executed\n") == 0) {
			cmdConfirmFlag = true;
			if (moveCmdFlag == true) {
				moveCmdCnt = 4;
				systemStat.sdata.robot_state = 3;
				moveCmdFlag = false;
			}	
		}
		cout << "moveCmdFlag: " << boolalpha << moveCmdFlag << endl;
	}
	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;
}


void Cobot::ReadData() {

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

bool Cobot::isMotionIdle() {
	return ((cmdConfirmFlag == true) && (systemStat.sdata.robot_state == 1));
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
	cout << "Command socket : " << CMD_CLIENT_FD_ << endl;
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
		cout << "Connect2Server return false" << endl;
		return false;
	}
	cout << "Cmd client connect to server..!!" << endl;

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
	cout << "Data socket : " << DATA_CLIENT_FD_ << endl;
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
		cout << "Connect2Server return false" << endl;
		return false;
	}
	cout << "Data client connect to server..!!" << endl;

	unsigned long arg = 1;
	return true;
}

bool Cobot::socketDataClose() {
	WSACleanup();
	return closesocket(DATA_CLIENT_FD_);
}

