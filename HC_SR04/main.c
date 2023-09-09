/**
 * File Name: HC_SR04
 * <Author>: Jie Li
 * <E-mail>: ssyjl7@nottingham.ac.uk
 * <Copyright>: Jie Li
 * <Date> 2023.09.09
 * University of Nottingham, Electrical and Electronic Engineering
 * OpenCV-Based PID Control Line Following Vehicle with Object Recognition and Reaction
 **/
#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>                            //Timer
#include <lcd.h>                                 //LCD headers from WiringPi
#define TRIG 23                                  //Associate pin 23 to TRIG
#define ECHO 24                                  //Associate pin 24 to ECHO

#define LCD_RS  3               //Register select pin
#define LCD_E   0               //Enable Pin
#define LCD_D4  6               //Data pin 4
#define LCD_D5  1               //Data pin 5
#define LCD_D6  5               //Data pin 6
#define LCD_D7  4               //Data pin 7

int main()
{
    float pulse_start,pulse_end,pulse_duration;
    float distance;
    int lcd;
    struct timeval start,end;
    long time;

    printf("Distance measurement in progress\n");

    wiringPiSetup();
    if (lcd = lcdInit (2, 16,4, LCD_RS, LCD_E ,LCD_D4 , LCD_D5, LCD_D6,LCD_D7,0,0,0,0)){
            printf ("lcdInit failed! \n");
            return -1 ;
    }

    pinMode(TRIG,OUTPUT);                  //Set pin as GPIO out
    pinMode(ECHO,INPUT);                //Set pin as GPIO in

    while ( 1 )
    {
        digitalWrite(TRIG, LOW);                 //Set TRIG as LOW
        printf("Waitng For Sensor To Settle\n");
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
            lcdPrintf(lcd, "Distance: ");  //Print the text on the LCD at the current cursor postion;
            lcdPrintf(lcd, "%.2f",distance);
        }
        else
        {
            printf("Out Of Range");
            lcdPosition(lcd,0,10);
            lcdPrintf(lcd,"ERROR");         //display out of range
        }
    }
}

