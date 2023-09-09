/** Project: Template matching  */

/**
 * File Name: Select_task
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * University of Nottingham, Electrical and Electronic Engineering
 * OpenCV-Based PID Control Line Following Vehicle with Object Recognition and Reaction
 **/

#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv_aee.hpp"
#include <wiringSerial.h>
using namespace std;
using namespace cv;
std::vector<std::vector<cv::Point>> contours;
//a series of vector that record the contours
std::vector<Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
cv::Mat isoframe(240, 320, CV_8U);
const std::string symbols[12] = { "1.png","2.png","3.png","4.png","5.png","6.png","7.png","8.png","9.png","10.png","11.png","12.png" };

int taskselect(Mat x) {
    int cnt = 0;
    Mat img = x;
    Mat frameHSV, kernel, imgGaus, FrameHSV;
    double areamax = 0, epsilon = 0, similarity = 0, maxsim = 0;

    cvtColor(img, frameHSV, COLOR_BGR2HSV);
    cv::inRange(frameHSV, Scalar(110, 63, 76), Scalar(168, 191, 188), frameHSV); // //Inrange to make the image to only have balck and white

    Mat src = frameHSV;
    findContours(src, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());  // Find the contours

    for (int i = 0; i<int(contours.size()); i++)
    {
        double area = abs(contourArea(contours[i]));
        if (areamax < area) areamax = area, cnt = i;
    }
    //cout << "contours.size(): " << contours.size() << endl;
    //cout << "cnt: " << cnt << endl;

    epsilon = 0.1 * arcLength(contours[cnt], true);
    approxPolyDP(contours[cnt], approx, epsilon, true);
    Mat isoframe_noblur;
    isoframe_noblur = transformPerspective(approx, src, 320, 240);
    //isoframe = transformPerspective(approx, src, 320, 240);
    GaussianBlur(isoframe_noblur, isoframe, Size(5,5), 10, 20);

    //          Find the best matching image
    int task = 0;
    for (int i = 0; i <= 11; i++)
    {
        Mat libsym = imread(symbols[i]);
        cv::inRange(libsym, Scalar(250, 0, 250), Scalar(255, 5, 255), libsym);
        similarity = compareImages(isoframe, libsym);
        if (similarity > maxsim) maxsim = similarity, task = i;
        printf("%f\n", similarity);
    }

    imshow("isoframe",isoframe);
    imshow("Template", Template);

    return task;
}

int main() {
    Mat img;
    int task = 0;
    img = imread("/home/pi/Desktop/TaskPhotoedByPi/5.png");

    task = taskselect(img) + 1;
    printf("task%d", task);
}
