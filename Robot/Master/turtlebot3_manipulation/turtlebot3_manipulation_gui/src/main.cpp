#include <stdio.h>
#include <stdlib.h>
#include <QtGui>
#include <QApplication>
#include "../include/turtlebot3_manipulation_gui/main_window.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <queue>
#include <string.h>

using namespace std;

// data getting from turtlebot
char curr[4] = "XXX";

// Ros communication
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
	memcpy(curr, msg->data.c_str(), sizeof(msg->data.c_str()));
}

/* ------------------------- main ----------------------------*/
int main(int argc, char **argv)
{
	QApplication app(argc, argv);
	turtlebot3_manipulation_gui::MainWindow w(argc, argv);

	//주소지 따른 행동
	float a[4] = {0.550, 0.500, -0.100, 0.500}; // 주소지 a가 행동할 방향
	float b[4] = {0.800, 0.250, 0.000, 0.900};	// 주소지 b가 행동할 방향
	float c[4] = {0.950, 0.650, -0.650, 0.750}; // 주소지 c가 행동할 방향
	// 치기
	float a_1[4] = {0.750, 0.500, -0.100, 0.200}; // a 박스 전달
	float b_1[4] = {0.700, 0.550, -0.250, 0.300}; // b 박스 전달
	float c_1[4] = {0.600, 0.650, -0.800, 1.000}; // c 박스 전달

	// 통신 init
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	queue<char> q; // data 전달받을 주소지 (queue)
	char prev[4] = "XXX";
	char end[4] = "END";
	char A = 'A';
	char B = 'B';
	char C = 'C';

	// application 보여주고 거기서 실행 log 보여주기
	w.show();
	w.on_btn_timer_start_clicked(); // 처음에 Qtimer 시작해야하기 때문에 클릭
	w.on_btn_init_pose_clicked();
	sleep(1);
	w.on_btn_gripper_close_clicked(); // start with gripper closed
	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

	while (1)
	{
		// COMMUNICATION
		ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
		ros::Rate loop_rate(1);
		printf("%s\n",curr);

		if (strcmp(end, curr) == 0)
			break;
			
		if (strcmp(prev, curr) != 0)
		{
			memcpy(prev, curr, sizeof(prev));
			q.push(curr[0]);
		}

		loop_rate.sleep();
		ros::spinOnce();

		ros::Rate callback_rate(1);
		ros::AsyncSpinner spinner(0);
		spinner.start(); // for multithread

		if (q.front() == A)
		{
			std::cout << "A start\n";
			//usleep(2300000);
                        //usleep(3000000);
			w.on_btn_send_joint_angle_clicked(a[0], a[1], a[2], a[3]);
			usleep(2400000);
			//usleep(1300000);
			w.on_btn_send_joint_angle_clicked(a_1[0], a_1[1], a_1[2], a_1[3]);
                        usleep(2400000);
			//usleep(1400000);
			w.on_btn_init_pose_clicked();
			std::cout << "a\n";

			q.pop();
		}

		else if (q.front() == B)
		{
			std::cout << "B start\n";
			//usleep(2300000);
                        //usleep(3000000);
			w.on_btn_send_joint_angle_clicked(b[0], b[1], b[2], b[3]);
                        usleep(2400000);
			//usleep(1300000);
			w.on_btn_send_joint_angle_clicked(b_1[0], b_1[1], b_1[2], b_1[3]);
                        usleep(2400000);
			//usleep(1400000);
			w.on_btn_init_pose_clicked();
			std::cout << "b\n";

			q.pop();
		}

		else if (q.front() == C)
		{
			std::cout << "C start\n";
			//usleep(2300000);
                        //usleep(3000000);
			w.on_btn_send_joint_angle_clicked(c[0], c[1], c[2], c[3]);
                        usleep(2400000);
			//usleep(1300000);
			w.on_btn_send_joint_angle_clicked(c_1[0], c_1[1], c_1[2], c_1[3]);
                        usleep(2400000);
			//usleep(1400000);
			w.on_btn_init_pose_clicked();
			std::cout << "c\n";

			q.pop();
		}
	}
        printf("Exit while\n");
	int result = app.exec();
	ros::waitForShutdown();
	return result;
}
