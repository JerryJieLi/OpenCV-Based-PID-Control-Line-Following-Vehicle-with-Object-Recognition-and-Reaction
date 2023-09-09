/** Project: Use track bar to read HSV value of a frame  */

/**
 * File Name: HSV_Detect
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * University of Nottingham, Electrical and Electronic Engineering
 * OpenCV-Based PID Control Line Following Vehicle with Object Recognition and Reaction
 **/
/** Project: Line following with HSV  */

/**
 * File Name: HSV_Detect
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * <Date> 2023.09.09
 * University of Nottingham, Electrical and Electronic Engineering
 * OpenCV-Based PID Control Line Following Vehicle with Object Recognition and Reaction
 **/
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

int main()
{
    cout << "start grabbing, press a key on Live window to terminate" << endl;
    Mat frame = imread("/home/pi/Desktop/TaskPhotoedByPi/0.png");
    Mat hsvImg;
    Mat imgThresholded;
    Mat imag_1;
    namedWindow("Trackbars", (640, 200));
    int hmin, hmax, smin, smax, vmin, vmax;
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
    while (1)
    {
        if (frame.empty())
        {
            cerr << "ERROR: unable to grab from the camera" << endl;
            break;
        } // end inner if

        int key = cv::waitKey(5);

        Mat imgHSV, mask;
        cvtColor(frame, imgHSV, COLOR_BGR2HSV);


        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);

        inRange(imgHSV, lower, upper, mask);

        imshow("frame", frame);

        imshow("ImageMask", mask);


    } // end while

    cout << "closing the camera" << endl;

    destroyAllWindows();
    cout << "bye!" << endl;

    return 0;
}

