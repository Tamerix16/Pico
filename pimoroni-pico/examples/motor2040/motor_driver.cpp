
#include <cstdio>
#include "pico/stdlib.h"
#include "motor2040.hpp"
using namespace motor;
 //initialise communication
Motor m = Motor(motor2040::MOTOR_A);
volatile int position = 0;
volatile bool READY = false;

void drive_forward()
{
    m.full_positive();
}
void soft_stop()
{
    m.coast();
}
int main() 
{
    stdio_init_all();
    // Your other initialization code
    m.init(); //initialise motor
    m.enable();
    int position = 0;
    char str[100];
    while (1) 
    {
        int c = getchar_timeout_us(0); // Check for a character immediately
        
        if (c!=PICO_ERROR_TIMEOUT)
        {
                position = 0;
            do
            {
                str[position] = (char)c;
                position++;
                READY = true;
            }while((c = getchar_timeout_us(0))!='\n'&& c!= PICO_ERROR_TIMEOUT);
            str[position] = '\0';
            printf("%c",str[0]);
            if(char(str[0])=='m')
            {
                drive_forward();
            }
            else
            {
                soft_stop();
            }
            
        }
           
    
        
    }
    return 0;
}

