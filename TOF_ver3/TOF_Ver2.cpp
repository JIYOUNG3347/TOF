// Library
#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <stdio.h>
#include <tchar.h>
#include "Serial.h"
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


// Command format
const char cmd_get_dis_amp[14] = { 0xF5, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9, 0xDF, 0xE8, 0x9E };


// Port and Baudrate
int LIDARPORT = 4;			// 장치관리자 확인
int BAUDRATE = 10000000;	// TOF 사용시 변경 X 


// Serial Port check
bool running;


static const uint32_t polynom = 0x04C11DB7;
static const uint32_t initValue = 0xFFFFFFFF;
static const uint32_t xorValue = 0x00000000;

bool checkPck(unsigned char* response_data, int calc_response_crc);
uint32_t calcCrc32Uint32(uint32_t crc, uint32_t data);
uint32_t calcCrc32_32(uint8_t* data, uint32_t size);
vector<int> arr(unsigned char* Data, int a, int b, float c);


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
				dis_arr = arr(crc_data, 1, 0, 8000.0);
				amp_arr = arr(crc_data, 3, 2, 2000.0);
			}
			catch (...)
			{
				continue;
			}


			Mat dis_mat = (Mat)dis_arr;
			Mat amp_mat = (Mat)amp_arr;
			Mat img;
		
			

			dis_mat.convertTo(dis_mat, CV_8UC1);
			amp_mat.convertTo(amp_mat, CV_8UC1);

			//cout << dis_mat_reshaped << endl;
			
			Mat dis_mat_reshaped = dis_mat.reshape(1, 60);
			Mat amp_mat_reshaped = amp_mat.reshape(1, 60);

			
			//normalize(dis_mat_reshaped, dis_mat_reshaped, 0, 1, NORM_MINMAX);
			//normalize(amp_mat_reshaped, amp_mat_reshaped, 0, 1, NORM_MINMAX);


			resize(dis_mat_reshaped, dis_mat_reshaped, Size(0, 0), 5, 5, INTER_CUBIC);
			resize(amp_mat_reshaped, amp_mat_reshaped, Size(0, 0), 5, 5, INTER_CUBIC);


			vconcat(dis_mat_reshaped, amp_mat_reshaped, img);

			imshow("img", img);

			waitKey(10);

			//cout << dis_mat_reshaped << endl;

			/*
			for (int i = 0; i < 9600; i++) {
				int data = dis_arr[i];
				cout << data << " ";
			}
			*/

		}

		cout << endl << endl;

	}

}




// CRC Check
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



// CRC checksum
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



// ARR
vector<int> arr(unsigned char* Data, int a, int b, float c) {
	vector<int> arr;
	float data;
	for (int i = 84; i < 38484; i = i + 4) {

		data = (Data[a + i] & 0xff) << 8 | (Data[b + i] & 0xff);
		if (data > c) {
			data = c;
		}
		data = (data / c) * 255;
		arr.push_back(data);

	}
	return arr;
}
