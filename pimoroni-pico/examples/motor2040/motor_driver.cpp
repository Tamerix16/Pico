
#include <cstdio>
#include "pico/stdlib.h"
#include "motor2040.hpp"
#include "pid.hpp"

using namespace motor;
using namespace encoder;
const pin_pair MOTOR_PINS = motor2040::MOTOR_A;
const pin_pair ENCODER_PINS = motor2040::ENCODER_A;
constexpr float GEAR_RATIO = 50.0f;
constexpr float COUNTS_PER_REV = 14.0f*GEAR_RATIO;
const Direction DIRECTION = NORMAL_DIR;
constexpr float SPEED_SCALE = 5.4f;
const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f/float(UPDATES);
constexpr float VEL_KP = 30.0f;   // Velocity proportional (P) gain
constexpr float VEL_KI = 0.0f;    // Velocity integral (I) gain
constexpr float VEL_KD = 0.4f;    // Velocity derivative (D) gain

Motor m = Motor(MOTOR_PINS, DIRECTION, SPEED_SCALE);
Encoder enc = Encoder(pio0, 0, ENCODER_PINS, PIN_UNUSED, DIRECTION, COUNTS_PER_REV, true);
PID vel_pid = PID(VEL_KP, VEL_KI, VEL_KD, UPDATE_RATE);


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
    enc.init();
    float speed_m1 = 0;
    int position = 0;
    char str[100];
    while (1) 
    {
        vel_pid.setpoint = speed_m1;
        Encoder::Capture capture = enc.capture();
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
            if(str[0] == 'w')
            {
                speed_m1 = (float)(str[1]-'0');
                vel_pid.setpoint = speed_m1;
                printf("%f",speed_m1);
            }
            else if(str[0] == 's')
            {
                speed_m1=(float)(-1*(str[1]-'0'));
                vel_pid.setpoint = speed_m1;
                printf("%f",speed_m1);
            }

            
        }
        float speed_adjust = vel_pid.calculate(capture.revolutions_per_second());
        m.speed(m.speed() + (speed_adjust * UPDATE_RATE));
        printf("Vel = %f\n ", capture.revolutions_per_second());
           
    
        
    }
    return 0;
}

