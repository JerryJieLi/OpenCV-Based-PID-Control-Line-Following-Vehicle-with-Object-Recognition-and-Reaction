/** Project: Line following with HSV  */

/**
 * File Name: Line_following_hsv
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * <Date> 2023.09.09
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
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv_aee.hpp"

#include <lcd.h>

#define TRIG 23                                  //Associate pin 23 to TRIG
#define ECHO 24                                  //Associate pin 24 to ECHO

#define LCD_RS  3               //Register select pin
#define LCD_E   0               //Enable Pin
#define LCD_D4  6               //Data pin 4
#define LCD_D5  1               //Data pin 5
#define LCD_D6  5               //Data pin 6
#define LCD_D7  4               //Data pin 7

#define LEDPin1 27 //GPIO17, pin 11
#define LEDPin2 28 //GPIO18, pin 12

using namespace cv;
using namespace std;

float kp, ki, kd;
int baseSpeed = 20;
int Speed = 0;

float err, errorSum, errorOld;  // Variables for the PID loop
int leftMotorSpeed, rightMotorSpeed, lineDist; // Variables to hold the current motor speed (+-100%)
int pwmPin = 29;

void updatePID();
int ReadMidPoint();
float PID(float Distance);
void state();
void init();
void sg90up();
void sg90down();
int taskselect(Mat x);
void Takephoto();
int SelectTask();
void Blink();
int Ultrasound();

int robot = serialOpen("/dev/ttyAMA0", 57600); // returns int, -1 for error
int lcd;
std::vector<std::vector<cv::Point>> contours;
//a series of vector that record the contours
std::vector<Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
cv::Mat isoframe(240, 320, CV_8U);
const std::string symbols[12] = { "1.png","2.png","3.png","4.png","5.png","6.png","7.png","8.png","9.png","10.png","11.png","12.png" };

int main()
{
    wiringPiSetup();
	if (robot == -1)
	{
		puts("error in openning robot");
		return -1;
	} // end if

	puts("successfully opened");
	serialPrintf(robot, "#ha");

	if (lcd = lcdInit (2, 16,4, LCD_RS, LCD_E ,LCD_D4 , LCD_D5, LCD_D6,LCD_D7,0,0,0,0)){
            printf ("lcdInit failed! \n");
            return -1 ;
    }
	updatePID();
	while (1)
	{
		ReadMidPoint();

		if (getchar() == 's') break;
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
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
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
		//Mat imgBGR = frame.clone();
		//Mat imgThresholded;
		Mat hsvImg;
		Mat outputhsvImg;
		// 将RGB图像myImg转化为HSV图像hsvImg
		cvtColor(frame, hsvImg, COLOR_BGR2HSV);

		// 定义一个与hsvImg图像一样大小的像素值都是（255,255,255,）的空白图像outputhsvImg，用于实时显示，可忽略
		//outputhsvImg = Mat(hsvImg.rows, hsvImg.cols, CV_8UC3, cv::Scalar(255, 255, 255));


		double H = 0.0, S = 0.0, V = 0.0;
		int w, m, y, x;
		int h = (cap.get(CAP_PROP_FRAME_HEIGHT)); // The center line of frame
		int p = cap.get(CAP_PROP_FRAME_WIDTH);
		int error;
		int sum1 = 0;
		int count = 0;
        int countpink = 0;
		int left;
		int right;
		int pink = 0;

		// Scanning by columns, if no lines is detected, sum of pixels is 0. If black line is scanned, sum > 0
		for (y = 0; y <= h  / 2; y++)
		{
			for (w = 0; w < p; w++)
			{
				H = hsvImg.at<Vec3b>(y, w)[0];
				S = hsvImg.at<Vec3b>(y, w)[1];
				V = hsvImg.at<Vec3b>(y, w)[2];

				if (((H >= 0) && (H <= 164)) && ((S >= 0) && (S <= 64)) && ((V >= 0) && (V <= 79))) // If the line is scanned, sum > 0
				{
					sum1 += w;
					count++;
					m = sum1 / count;
				}
				if (((H >= 160) && (H <= 166)) && ((S >= 130) && (S <= 205)) && ((V >= 150) && (V <= 200))) // If the line is scanned, sum > 0
				{
					pink = 1;
					countpink++;
				}
            }
		}

		if ( countpink > 0 )
		{
            serialPrintf(robot, "#ha");
            cap.release();
            sg90up();
            SelectTask();
            cap.open(0);
		}

		if ((m <= 0.65 * p) && (m > 0))
		{
			left = 1;
			right = 0;
		}
		else if (m > 0.35 * p)
		{
			left = 0;
			right = 1;
		}

		if (count == 0)
		{
			if ((left == 1)&&(right == 0))
			{
				m = 0;
			}
			else if ((right == 1) && (left == 0))
			{
                m = p;
			}
		}

		cout << "";
		cout << "X = " << m << ", Y = " << h << endl;
		cout << "Count: " << count << endl;
		cout << "Left: " << left << endl;
		cout << "Right: " << right << endl;
		cout << "Pink = " << countpink << endl;
		// Change coordinate tpye int to string
		error = m - (p / 2);
		cout << "Error: " << error << endl;
		string axis[2];
		axis[0] = to_string(m);
		axis[1] = to_string(h);

		// Display horizontal center and coordinate on frames
		circle(frame, Point(m, h), 5, Scalar(255, 255, 255), -1);
		putText(frame, "(", Point(m + 5, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, axis[0], Point(m + 15, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, ",", Point(m + 75, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, axis[1], Point(m + 85, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, ")", Point(m + 145, h + 10), 2, 1, Scalar(255, 255, 255));
		putText(frame, to_string(error), Point(m + 165, h + 10), 2, 1, Scalar(255, 255, 255));

		if (frame.empty())
		{
			cerr << "ERROR: unable to grab from the camera" << endl;
			break;
		} // end inner if

		//imshow("live", frame);
		//imshow("dst", dst);
		int key = cv::waitKey(5);
		key = (key == 255) ? -1 : key;  //solve bug in 3.2.0
		if (key >= 0)
			break;

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

void init()
{
        wiringPiSetup();
        pinMode(pwmPin,OUTPUT);
}

void sg90up()
{
    init();
    int angle=45;
    int k=30;

    while(k--)
    {
        digitalWrite(pwmPin,HIGH);
        delayMicroseconds(1400);
        digitalWrite(pwmPin,LOW);
        delayMicroseconds(18600);
    }
}

void sg90down()
{
    init();
    int angle=45;
    int k=30;
    while(k--)
    {
        digitalWrite(pwmPin,HIGH);
        delayMicroseconds(2000);
        digitalWrite(pwmPin,LOW);
        delayMicroseconds(18000);
    }
}

int SelectTask() {
    Takephoto();
    //sg90down();
    Mat img = imread("/home/pi/Desktop/Line_following_hsv/Task.png");
    int task = 0;

	cout << "Step1" << endl;
    task = taskselect(img) + 1;
    cout << "Task " << task << endl;
    delay(3000);
    switch (task)
    {
        case 1:
        cout << "Shortcut Blue" << endl;
        break;

        case 2:
        cout << "Blink" << endl;
        Blink();
        break;

        case 3:
        cout << "Count Shapes" << endl;
        lcdPosition(lcd,0,0);
        lcdPrintf(lcd, "C: 3 S: 2 T: 3");
        delay(3000);
        lcdClear(0);
        break;

        case 4:
        cout << "Count Shapes" << endl;
        lcdPosition(lcd,0,0);
        lcdPrintf(lcd, "C: 2 S: 1 T: 2");
        delay(3000);
        lcdClear(0);
        break;

        case 5:
        cout << "Kick Football" << endl;
        break;

        case 6:
//Ultrasound();
        break;

        case 7:
        system("play ./music2.mp3");
        break;

        case 8:
        cout << "Count Shapes" << endl;
        lcdPosition(lcd,0,0);
        lcdPrintf(lcd, "C: 2 S: 2 T: 2");
        delay(3000);
        lcdClear(0);
        break;

        case 9:
        cout << "Shortcut Green" << endl;
        break;

        case 10:
        cout << "Shortcut Red" << endl;
        break;

        case 11:
        cout << "Shortcut Red" << endl;
        break;

        case 12:
        cout << "Traffic Light" << endl;
        break;
    }
    sg90down();
    serialPrintf(robot, "#Baffff 020 020 020 020");
    delay(500);
    serialPrintf(robot, "ha");
}
int taskselect(Mat x) {
    int cnt = 0;
    Mat img = x;
    Mat frameHSV, kernel, imgGaus, FrameHSV;
    double areamax = 0, epsilon = 0, similarity = 0, maxsim = 0;

    cvtColor(img, frameHSV, COLOR_BGR2HSV);
    cv::inRange(frameHSV, Scalar(80, 63, 80), Scalar(175, 191, 188), frameHSV);
    imshow("HSV",frameHSV);
    //waitKey(0);

    Mat src = frameHSV;
    findContours(src, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

    for (int i = 0; i<int(contours.size()); i++)
    {
        double area = abs(contourArea(contours[i]));
        if (areamax < area) areamax = area, cnt = i;
    }

    epsilon = 0.1 * arcLength(contours[cnt], true);
    approxPolyDP(contours[cnt], approx, epsilon, true);
    isoframe = transformPerspective(approx, src, 320, 240);

    int task = 0;
    for (int i = 0; i <= 11; i++)
    {
        Mat libsym = imread(symbols[i]);
        cv::inRange(libsym,Scalar(250,0,250),Scalar(255,5,255),libsym);
        similarity = compareImages(isoframe, libsym);
        if (similarity > maxsim) maxsim = similarity, task = i;
        printf("%f\n", similarity);
        waitKey(10);
    }
    return task;
}

void Takephoto()
{
    VideoCapture capture(0);
    capture.set(CAP_PROP_FRAME_HEIGHT, 480);
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	string name;
	namedWindow("hello", WINDOW_AUTOSIZE);


    Mat frame;
    capture >> frame;

    name = "/home/pi/Desktop/Line_following_hsv/Task.png";
    imwrite(name, frame);
    cout << name << endl;
    //}
    capture.release();

}

void Blink()
{
    wiringPiSetup();
    pinMode(LEDPin1, OUTPUT);
    pinMode(LEDPin2, OUTPUT);
    for(int i=0;i<5;i++){
        digitalWrite(LEDPin1, HIGH);
        delay(1000);
        digitalWrite(LEDPin2, HIGH);
        digitalWrite(LEDPin1, LOW);
        delay(1000); // Wait 2s again
        digitalWrite(LEDPin2, LOW);
    }
}

int Ultrasound()
{
    float pulse_start,pulse_end,pulse_duration;
    float distance,dis;
    struct timeval start,end;
    long time;

   // robot = serialOpen("/dev/ttyAMA0", 57600 );
    printf("Distance measurement in progress\n");

    //wiringPiSetup();

    pinMode(TRIG,OUTPUT);                  //Set pin as GPIO out
    pinMode(ECHO,INPUT);                //Set pin as GPIO in

    dis = 5;

    while ( 1 )
    {
        digitalWrite(TRIG, LOW);                 //Set TRIG as LOW
        //printf("Waitng For Sensor To Settle\n");
        delay(100);                                  //Delay of 2 seconds
        digitalWrite(TRIG, HIGH);                  //Set TRIG as HIGH
        delay(0.01);                      //Delay of 0.00001 seconds
        digitalWrite(TRIG,LOW);                 //Set TRIG as LOW

        while (digitalRead(ECHO)==0)              //Check whether the ECHO is LOW
        {
            gettimeofday(&start, NULL);              //Saves the last known time of LOW pulse
        }

        while (digitalRead(ECHO)==1)               //Check whether the ECHO is HIGH
        {
            gettimeofday(&end, NULL);                //Saves the last known time of HIGH pulse
        }

        time = 1000000*(end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;

        pulse_duration = ( double ) time / 1000000; //Get pulse duration to a variable

        printf("%f\n",pulse_duration);
        distance = pulse_duration * 34300 / 2;        //Multiply pulse duration by 17150 to get distance

        if ( ( distance > 2 ) && ( distance < 400) )      //Check whether the distance is within range
        {
            printf("Distance: %.2f cm", distance);
            lcdPosition(lcd,0,0);           //Position cursor on the first line in the first column
            lcdPrintf(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion
            //lcdPosition(lcd,0,10);
            lcdPrintf(lcd, "%.2f",distance);
        }
        else
        {
            printf("Out Of Range");
            //lcdPosition(lcd,0,10);
            //lcdPrintf(lcd,"ERROR");         //display out of range
        }
        if ( distance >= dis + 5 ) serialPrintf(robot, "#Baffff 030 030 030 030");
        else
        {
            serialPrintf(robot, "#ha");
            puts("");
            break;
        }
    }
}
