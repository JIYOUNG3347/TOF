/*Library*/
#define _CRT_SECURE_NO_WARNINGS

// Includes
#include <stdio.h>
#include <tchar.h>
#include <string>
#include <iostream>
#include <list>
#include <vector>
#include <queue>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>

// opencv Includes
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// crc checksum & Serial Includes
#include "crc_calc.h"
#include "Serial.h"

// namespace 
using namespace std;
using namespace cv;
using namespace ComLib;



// Command format
const char cmd_get_dis_amp[14] = { 0xF5, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9, 0xDF, 0xE8, 0x9E };


// Port and Baudrate
int LIDARPORT = 4;			// Check port
int BAUDRATE = 10000000;	// Do not change values


// Global variables
bool running;

// function declaration
bool checkPck(unsigned char* response_data, int calc_response_crc);		// CRC Check function
vector<int> arr(unsigned char* Data, int a, int b, float c);			// Split data to Distance data and Amplitude data function, outlier handling, Normalize

constexpr float CONFIDENCE_THRESHOLD = 0;
constexpr float NMS_THRESHOLD = 0.6;
constexpr int NUM_CLASSES = 1;


// colors for bounding boxes
const cv::Scalar colors[] = {
	{0, 255, 255},
	{255, 255, 0},
	{255, 255, 0},
	{255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);



int main()
{
	std::vector<std::string> class_names;
	{
		std::ifstream class_file("obj.txt");
		if (!class_file)
		{
			std::cerr << "failed to open classes.txt\n";
			return 0;
		}

		std::string line;
		while (std::getline(class_file, line))
			class_names.push_back(line);
	}

	

	// variables
	unsigned char response_data[38488] = {};							// Response data
	int rxdata;															// Resposne data start byte
	bool cmp;															// CRC Check 

	vector<int> dis_arr;												// Distance data array
	vector<int> amp_arr;												// Amplitude ata array

	auto net = cv::dnn::readNetFromDarknet("yolo-obj.cfg", "yolo-obj_last.weights");
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	auto output_names = net.getUnconnectedOutLayersNames();

	Mat frame, blob;
	vector<cv::Mat> detections;

	// Set Serial Port, BAUDRATE
	TSerialPort ser;
	ser.Open(LIDARPORT, BAUDRATE, 10);

	// Serial Port Open check
	if (ser.IsOpen()) {
		cout << "Serial port is opened" << endl;
		running = true;
	}
	else
		running = false;


	while (running) {

		ser.WriteBuffer(cmd_get_dis_amp, 14);							// Command format
		ser.ReadBuffer(response_data, 38488, -1);						// Resposnse format -> start byte (1), type (1), length (2), header data (80), ~, crc (4)

		
		rxdata = response_data[0];										// start byte


		// Start byte check
		if (rxdata == 0xfa) {

			unsigned char crc_data[38484] = { 0, };
			memcpy(crc_data, response_data, 38484);




			// CRC packet check
			uint32_t size = sizeof(crc_data) / sizeof(char);
			uint32_t crc_result = CrcCalc::calcCrc32_32(crc_data, size);

			cmp = checkPck(response_data, crc_result);

			if (cmp != true) continue;


			// Split data to Distance data and Amplitude data
			try {
				dis_arr = arr(crc_data, 1, 0, 8000.0);
				amp_arr = arr(crc_data, 3, 2, 2000.0);
			}
			catch (...)
			{
				continue;
			}




			// Mat Data
			Mat dis_mat = (Mat)dis_arr;
			Mat amp_mat = (Mat)amp_arr;
			Mat img;

			// Convert to uint8_t Type
			/*
			If you don't do this first, resize function will throw an error.
			*/
			dis_mat.convertTo(dis_mat, CV_8UC1);
			amp_mat.convertTo(amp_mat, CV_8UC1);


			Mat dis_mat_reshaped = dis_mat.reshape(1, 60);
			Mat amp_mat_reshaped = amp_mat.reshape(1, 60);

			resize(dis_mat_reshaped, dis_mat_reshaped, Size(0, 0), 5, 5, INTER_CUBIC);
			resize(amp_mat_reshaped, frame, Size(150, 150), 0, 0, INTER_CUBIC);

			auto total_start = std::chrono::steady_clock::now();
			cv::dnn::blobFromImage(frame, blob, 1 / 255.f, cv::Size(150, 150), cv::Scalar(), true, false, CV_32F);
			net.setInput(blob);

			auto dnn_start = std::chrono::steady_clock::now();
			net.forward(detections, output_names);
			auto dnn_end = std::chrono::steady_clock::now();

			std::vector<int> indices[NUM_CLASSES];
			std::vector<cv::Rect> boxes[NUM_CLASSES];
			std::vector<float> scores[NUM_CLASSES];

			for (auto& output : detections)
			{
				const auto num_boxes = output.rows;
				for (int i = 0; i < num_boxes; i++)
				{
					auto x = output.at<float>(i, 0) * frame.cols;
					auto y = output.at<float>(i, 1) * frame.rows;
					auto width = output.at<float>(i, 2) * frame.cols;
					auto height = output.at<float>(i, 3) * frame.rows;
					cv::Rect rect(x - width / 2, y - height / 2, width, height);

					for (int c = 0; c < NUM_CLASSES; c++)
					{
						auto confidence = *output.ptr<float>(i, 5 + c);
						if (confidence >= CONFIDENCE_THRESHOLD)
						{
							boxes[c].push_back(rect);
							scores[c].push_back(confidence);
						}
					}
				}
			}

			for (int c = 0; c < NUM_CLASSES; c++)
				cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, NMS_THRESHOLD, indices[c]);

			for (int c = 0; c < NUM_CLASSES; c++)
			{
				for (size_t i = 0; i < indices[c].size(); ++i)
				{
					const auto color = colors[c % NUM_COLORS];

					auto idx = indices[c][i];
					const auto& rect = boxes[c][idx];
					cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);

					std::ostringstream label_ss;
					label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
					auto label = label_ss.str();

					int baseline;
					auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.1, 1, &baseline);
					cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
					cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar(255, 255, 255));
				}
			}

			auto total_end = std::chrono::steady_clock::now();

			float inference_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(dnn_end - dnn_start).count();
			float total_fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count();
			std::ostringstream stats_ss;
			stats_ss << std::fixed << std::setprecision(2);
			stats_ss << "FPS: " << total_fps;
			auto stats = stats_ss.str();

			int baseline;
			auto stats_bg_sz = cv::getTextSize(stats.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
			cv::putText(frame, stats.c_str(), cv::Point(0, stats_bg_sz.height + 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 255, 255));
			resize(frame, frame, Size(0, 0), 5, 5, INTER_CUBIC);

			cv::namedWindow("output");
			cv::imshow("output", frame);



			if (waitKey(1) == ('q'))
				break;

		}
	}

}




// CRC Check function
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


// Split data to Distance data and Amplitude data function, outlier handling,  Normalize
vector<int> arr(unsigned char* Data, int a, int b, float c) {
	vector<int> arr;
	float data;
	for (int i = 84; i < 38484; i = i + 4) {

		data = (Data[a + i] & 0xff) << 8 | (Data[b + i] & 0xff);
		// outlier handling
		if (data > c) {
			data = c;
		}
		// Normalize between 0 and 255
		data = (data / c) * 255;

		arr.push_back(data);

	}
	return arr;
}
