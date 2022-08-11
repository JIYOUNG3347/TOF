// TOF_Ver2.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <stdio.h>
#include <tchar.h>
#include "Serial.h"	// Library described above
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include<cstring>
#include <cstdio>
#include <stdlib.h>
#include <string.h>

using namespace std;
using namespace cv;


const char cmd_get_dis_amp[14] = { 0xF5, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9, 0xDF, 0xE8, 0x9E };
int LIDARPORT = 4;
string Arduino = "COM5";
int BAUDRATE = 10000000;
bool running;
static const uint32_t polynom = 0x04C11DB7;
static const uint32_t initValue = 0xFFFFFFFF;
static const uint32_t xorValue = 0x00000000;

bool checkPck(unsigned char* response_data, int calc_response_crc);
uint32_t calcCrc32Uint32(uint32_t crc, uint32_t data);
uint32_t calcCrc32_32(uint8_t* data, uint32_t size);
vector<int> arr(unsigned char* Data, int a, int b);


int main()
{
	unsigned char response_data[38488] = {};
	int rxdata;
	uint32_t size;
	uint32_t crc_result;
	bool cmp;
	vector<int> dis_arr;
	vector<int> amp_arr;

	TSerialPort ser;
	ser.Open(LIDARPORT, BAUDRATE, 10);

	if (ser.IsOpen()) {
		cout << "Serial port is opened" << endl;
		running = true;
	}
	else
		running = false;


	while (running) {

		ser.WriteBuffer(cmd_get_dis_amp, 14);
		ser.ReadBuffer(response_data, 38488, -1);


		rxdata = response_data[0];


		if (rxdata == 0xfa) {
			unsigned char crc_data[38484] = { 0, };
			memcpy(crc_data, response_data, 38484);


			// crc
			size = sizeof(crc_data) / sizeof(char);
			crc_result = calcCrc32_32(crc_data, size);

			cmp = checkPck(response_data, crc_result);



			/*
			if (cmp == 1) {
				cout << "CRC Check True" << endl;
			}
			else
			{
				cout << "CRC Check False" << endl;
			}
			*/

			if (cmp != true) continue;

			try {
				dis_arr = arr(crc_data, 1, 0);
				amp_arr = arr(crc_data, 3, 2);
			}
			catch (...)
			{
				continue;
			}


			Mat dis_mat = (Mat)dis_arr;
			Mat amp_mat = (Mat)amp_arr;


			Mat dis_mat_reshaped = dis_mat.reshape(0, 60);
			Mat amp_mat_reshaped = amp_mat.reshape(0, 60);
			//cout << dis_mat_reshaped << endl;

			/*
			for (int i = 0; i < 9600; i++) {
				int data = dis_arr[i];
				cout << data << " ";
			}
			*/

			cout << "DIS Data: " << dis_mat_reshaped.size() << " " << dis_mat_reshaped.channels() << endl;
			cout << "AMP Data: " << amp_mat_reshaped.size() << " " << amp_mat_reshaped.channels() << endl;

			int width = amp_mat_reshaped.cols;
			int height = amp_mat_reshaped.rows;
			int bpp = amp_mat_reshaped.channels();

			int imagesize = width * height * bpp;
			uint8_t* buffer = new uint8_t[imagesize];
			memcpy(buffer, dis_mat_reshaped.data, imagesize);

			Mat gray(height, width, CV_8UC1, buffer);

			resize(gray, gray, Size(0,0), 5, 5, INTER_CUBIC);
			imshow("image", gray);

			waitKey(10);

		}

		cout << endl << endl;

	}

}



bool checkPck(unsigned char* response_data, int calc_response_crc)
{
	int bit1, bit2, bit3, bit4;
	bool cmp;

	bit4 = (calc_response_crc >> 24) & 0xff;
	bit3 = (calc_response_crc >> 16) & 0xff;
	bit2 = (calc_response_crc >> 8) & 0xff;
	bit1 = calc_response_crc & 0xff;
	cmp = (bit4 == response_data[38487]) & (bit3 == response_data[38486]) & (bit2 == response_data[38485]) & (bit1 == response_data[38484]);

	return cmp;
}




uint32_t calcCrc32Uint32(uint32_t crc, uint32_t data)
{
	int32_t i;

	crc = crc ^ data;

	for (i = 0; i < 32; i++)
	{
		if (crc & 0x80000000)
		{
			crc = (crc << 1) ^ polynom;
		}
		else
		{
			crc = (crc << 1);
		}
	}
	return(crc);
}


uint32_t calcCrc32_32(uint8_t* data, uint32_t size)
{
	uint32_t crc = initValue;

	for (uint32_t i = 0; i < size; i++)
	{
		crc = calcCrc32Uint32(crc, data[i]);
	}
	return crc ^ xorValue;
}


vector<int> arr(unsigned char* Data, int a, int b) {
	vector<int> arr;
	int data;
	for (int i = 84; i < 38484; i = i + 4) {

		data = (Data[a + i] & 0xff) << 8 | (Data[b + i] & 0xff);
		arr.push_back(data);

	}
	return arr;
}
// 프로그램 실행: <Ctrl+F5> 또는 [디버그] > [디버깅하지 않고 시작] 메뉴
// 프로그램 디버그: <F5> 키 또는 [디버그] > [디버깅 시작] 메뉴

// 시작을 위한 팁: 
//   1. [솔루션 탐색기] 창을 사용하여 파일을 추가/관리합니다.
//   2. [팀 탐색기] 창을 사용하여 소스 제어에 연결합니다.
//   3. [출력] 창을 사용하여 빌드 출력 및 기타 메시지를 확인합니다.
//   4. [오류 목록] 창을 사용하여 오류를 봅니다.
//   5. [프로젝트] > [새 항목 추가]로 이동하여 새 코드 파일을 만들거나, [프로젝트] > [기존 항목 추가]로 이동하여 기존 코드 파일을 프로젝트에 추가합니다.
//   6. 나중에 이 프로젝트를 다시 열려면 [파일] > [열기] > [프로젝트]로 이동하고 .sln 파일을 선택합니다.
