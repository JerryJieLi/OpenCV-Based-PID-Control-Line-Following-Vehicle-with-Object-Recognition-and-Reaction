/** Project: Template matching  */

/**
 * File Name: HSV_Detect
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * University of Nottingham, Electrical and Electronic Engineering
 * OpenCV-Based PID Control Line Following Vehicle with Object Recognition and Reaction
 **/

#include <stdio.h>
#include <wiringSerial.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>
#include <wiringPi.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

//int robot;
float kp, ki, kd;
int baseSpeed = 20;
int Speed = 0;

float err, errorSum, errorOld;  // Variables for the PID loop
int leftMotorSpeed, rightMotorSpeed, lineDist; // Variables to hold the current motor speed (+-100%)

void updatePID();
int ReadMidPoint();
float PID(float Distance);
void state();

int robot = serialOpen("/dev/ttyAMA0", 57600); // returns int, -1 for error

int main()
{
	if (robot == -1)
	{
		puts("error in openning robot");
		return -1;
	} // end if

	puts("successfully opened");
	serialPrintf(robot, "#ha");

	//update the following command
	while (1)
	{
		updatePID();
		ReadMidPoint();


		if (getchar()) break;
	}

	serialPrintf(robot, "#ha");

	serialClose(robot);

	return 0;
}

void updatePID()
{
	cout << "Kp = ";
	cin >> kp;
	cout << "Ki = ";
	cin >> ki;
	cout << "Kd = ";
	cin >> kd;
}

int ReadMidPoint()
{
	VideoCapture cap(0);
	Mat frame;
	cap.set(CAP_PROP_FPS, 5.5);
	cap.set(CAP_PROP_FRAME_HEIGHT, 320);
	cap.set(CAP_PROP_FRAME_WIDTH, 480);
	if (!cap.isOpened())
	{
		cerr << "ERROR: unable to open the camera" << endl;
		return 0;
	} // end if

	cout << "start grabbing, press a key on Live window to terminate" << endl;
	while (1)
	{
		cap >> frame;
		//Mat frame;
		Mat imgBGR = frame.clone();
		Mat imgThresholded;

		Mat dst;
		//inRange(imgBGR, Scalar(0, 128, 128), Scalar(127, 255, 255), imgThresholded); //黄色
		//inRange(imgBGR, Scalar(0, 0, 128), Scalar(127, 127, 255), imgThresholded); //红色
		//inRange(imgBGR, Scalar(128, 0, 0), Scalar(255, 127, 127), imgThresholded); //蓝色
		threshold(frame, dst, 50, 255, THRESH_BINARY_INV);  //Binarization of frames

		int nRows = dst.rows;
		int nCols = dst.cols;
		int w, m, y, x;
		int h = (cap.get(CAP_PROP_FRAME_HEIGHT)); // The center line of frame
		int p = cap.get(CAP_PROP_FRAME_WIDTH);
		int error;
		int sum1 = 0;
		int count = 0;
		int left;
		int right;

		// Scanning by columns, if no lines is detected, sum of pixels is 0. If black line is scanned, sum > 0
		for (y = 0; y <= (h / 2); y++)
		{
			for (w = 0; w < nCols; w++)
			{
				int sum = 0;

				uchar* pRow = dst.ptr<uchar>(h / 2, w); // Address of pixels
				sum += (int)(*pRow);

				if (sum > 0)  // If the line is scanned, sum > 0
				{
					sum1 += w;
					count++;
					m = sum1 / count;
				}
			}
		}
		if ((m <= 0.6 * p) && (m > 0))
		{
			left = 1;
			right = 0;
		}
        else if ( m > 0.4 * p)
        {
            left = 0;
            right = 1;
        }

		if (count == 0)
		{
		    if (left == 1)
			{
                m = 0;
			}
			if (right == 1)
			{
                m = p;
			}
		}

		cout << "";
		cout << "X = " << m << ", Y = " << h << endl;
		cout << "Count: " << count << endl;
		cout << "Left: " << left << endl;
		// Change coordinate tpye int to string
		error = m - (cap.get(CAP_PROP_FRAME_WIDTH) / 2);
		string axis[2];
		axis[0] = to_string(m);
		axis[1] = to_string(h);

		// Display horizontal center and coordinate on frames
		circle(frame, Point(m, h), 5, Scalar(255, 255, 255), -1);
		/*putText(frame, "(", Point(m + 5, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, axis[0], Point(m + 15, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, ",", Point(m + 75, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, axis[1], Point(m + 85, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, ")", Point(m + 145, h + 10), 2, 1, Scalar(255, 255, 255));*/
		putText(frame, to_string(error), Point(m + 165, h + 10), 2, 1, Scalar(255, 255, 255));

		if (frame.empty())
		{
			cerr << "ERROR: unable to grab from the camera" << endl;
			break;
		} // end inner if

		//imshow("live", frame);
		imshow("dst", dst);
		int key = cv::waitKey(5);
		key = (key == 255) ? -1 : key;  //solve bug in 3.2.0
		if (key >= 0)
			break;
        //if (getchar() == 's') break;

		Speed = PID(error);
		leftMotorSpeed = baseSpeed + Speed;
		rightMotorSpeed = baseSpeed - Speed;
		state();
	} // end while

	cout << "closing the camera" << endl;
	cap.release();
	destroyAllWindows();
}

float PID(float Distance)
{
	// PID loop
	errorOld = err;        // Save the old error for differential component
	err = Distance;  // Calculate the error in position
	errorSum += err;

	float proportional = err * kp;  // Calculate the components of the PID

	float integral = errorSum * ki;

	float differential = (err - errorOld) * kd;

	float output = proportional + integral + differential;  // Calculate the result
	//cout << integral;

	return output;
}

void state()
{
	if ((leftMotorSpeed >= 0) && (rightMotorSpeed >= 0))
	{
		serialPrintf(robot, "#Baffff %d %d %d %d", leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
	}
	else if ((leftMotorSpeed < 0) && (rightMotorSpeed > 0))
	{
		leftMotorSpeed = abs(leftMotorSpeed);
		serialPrintf(robot, "#Barrff %d %d %d %d", leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
	}
	else if ((leftMotorSpeed > 0) && (rightMotorSpeed < 0))
	{
		rightMotorSpeed = abs(rightMotorSpeed);
		serialPrintf(robot, "#Baffrr %d %d %d %d", leftMotorSpeed, leftMotorSpeed, rightMotorSpeed, rightMotorSpeed);
	}
}
