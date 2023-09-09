#include<stdio.h>
#include<wiringPi.h>
#include<softPwm.h>

int pwmPin = 29;

void init();
int main()
{
    init();
    softPwmWrite(pwmPin, 6);
    delay(3000);
    softPwmWrite(pwmPin, 15);
    delay(3000);

        //return 0;
}


void init()
{
        wiringPiSetup();
        softPwmCreate(pwmPin, 0, 200);
}
