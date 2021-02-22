#include "pch.h"
#include "Cobot.h"
#include <Ws2tcpip.h>
using namespace rb;

#define PACKET_SIZE 4000

//SOCKET data_sock_;
//SOCKET cmd_sock_;
thread proc;
Cobot::Cobot() {
	memset(&systemStat, 0, sizeof(systemSTAT));
	systemStat.sdata.program_mode = -1;
	systemStat.sdata.robot_state = -1;
}

Cobot::~Cobot() {
	//closesocket(cmd_sock_);
	proc.join();
	socketCmdClose();
	
}

bool Cobot::ConnectToCB(string ip) {
	bool ret = isValidIP(ip);
	if (ret) {
		//deep copy
		ip_address_ = ip;
		cout << ip_address_ << endl;
		if (socketCmdCom(ip_address_)) {
			proc = thread(&Cobot::readyCmdCom, this);
		}
		
		//proc.join();
		
		return true;
	}
	return false;
}

string Cobot::Version() {
	cout << __RB_VERSION__ << endl;
	return __RB_VERSION__;
}

void Cobot::CobotInit() {
	string msg = "mc jall init";
	cout << msg << endl;
	cmdConfirmFlag = false;
	if (!WSAGetLastError()) {
		send(CMD_CLIENT_FD_, msg.c_str(), msg.length(), 0);
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
//
//void Cobot::MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc) {
//	QString text;
//	text.sprintf("jointall %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3; //run
//}
//void Cobot::MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd, float acc) {
//	QString text;
//	text.sprintf("movetcp %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, x, y, z, rx, ry, rz);
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3; //run
//}
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
//void Cobot::MoveJointBlend_Clear() {
//	QString text;
//	text.sprintf("blend_jnt clear_pt");
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//}
//void Cobot::MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd, float acc) {
//	QString text;
//	text.sprintf("blend_jnt add_pt %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", spd, acc, joint1, joint2, joint3, joint4, joint5, joint6);
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3; //run
//}
//void Cobot::MoveJointBlend_MovePoint() {
//	QString text;
//	text.sprintf("blend_jnt move_pt");
//	moveCmdFlag = true;
//	cmdConfirmFlag = false;
//	cmdSocket.write(text.toStdString().c_str(), text.toStdString().length());
//	systemStat.sdata.robot_state = 3;
//}
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
//void data_recv() {
//	char buffer[PACKET_SIZE] = {}; //char 생성
//	string cmd; //string 생성
//	while (!WSAGetLastError()) {
//		ZeroMemory(&buffer, PACKET_SIZE); //buffer 비우기
//		recv(data_sock_, buffer, PACKET_SIZE, 0); //데이터받아오기
//		cmd = buffer; //buffer의값이 cmd에 들어갑니다
//		//if (cmd == "hi") break; //cmd의값이 "exit"일경우 데이터받아오기'만' 종료
//		cout << "데이터 받은 메세지: " << buffer << endl;
//	}
//}


void Cobot::readyCmdCom() {
	char buffer[PACKET_SIZE] = {}; //char 생성
	string cmd; //string 생성
	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;
	while (!WSAGetLastError()) {
		ZeroMemory(&buffer, PACKET_SIZE); //buffer 비우기
		//ioctlsocket()
		recv(CMD_CLIENT_FD_, buffer, PACKET_SIZE, 0); //데이터받아오기
		cmd = buffer; //buffer의값이 cmd에 들어갑니다
		//if (cmd == "hi") break; //cmd의값이 "exit"일경우 데이터받아오기'만' 종료
		cout << "커맨드 받은 메세지: " << cmd << endl;
		if (cmd.compare("The command was executed\n") == 0) {
			cmdConfirmFlag = true;
			if (moveCmdFlag == true) {
				moveCmdCnt = 4;
				systemStat.sdata.robot_state = 3;
				moveCmdFlag = false;
			}
		}
	}	
	socketCmdClose();
	cout << "WSAGetLastError(): " << WSAGetLastError() << endl;
	
}
bool Cobot::isValidIP(string ip) {
	stringstream ss(ip);
	string item;

	int iter = 0;
	while (getline(ss, item, '.')) {
		int number = stoi(item);
		if (number < 0) {
			throw "Error: negative value";
			return false;
		}
		else if (number > 255) {
			throw "Error: 255";
			return false;
		}
		iter++;
	}

	if (iter == 4) {
		return true;
	}
	else {
		throw "Error: 4";
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
	cout << "createSocket : " << CMD_CLIENT_FD_ << endl;
	if (CMD_CLIENT_FD_ == -1) {
		//exception
	}
	inet_pton(AF_INET, ip.c_str(), &ClientAddr.sin_addr.s_addr);

	ClientAddr.sin_family = AF_INET;
	ClientAddr.sin_port = htons(CMD_PORT);

	int optval = 1;
	const char optval2 = (const char)optval;

	setsockopt(CMD_CLIENT_FD_, SOL_SOCKET, SO_REUSEADDR, &optval2, sizeof(optval));
	//setsockopt(LAN_CLIENT_FD, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
	setsockopt(CMD_CLIENT_FD_, IPPROTO_TCP, TCP_NODELAY, &optval2, sizeof(optval));

	if (connect(CMD_CLIENT_FD_, (struct sockaddr*)&ClientAddr, sizeof(ClientAddr)) < 0){
		cout << "connect : " << CMD_CLIENT_FD_ << endl;
		cout << "Connect2Server return false" << endl;
		return false;
	}
	cout << "Client connect to server..!!" << endl;

	unsigned long arg = 1;
	
	return ioctlsocket(CMD_CLIENT_FD_, FIONBIO, &arg) == 0;
}

bool Cobot::socketCmdClose() {
	WSACleanup();
	return closesocket(CMD_CLIENT_FD_);
}


