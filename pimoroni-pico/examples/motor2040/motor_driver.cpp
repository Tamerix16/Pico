
#include <cstdio>
#include "pico/stdlib.h"
#include "motor2040.hpp"
#include "pid.hpp"
// motor_a = left
//motor_b = right
//motor_c = up/down
//motor_d = gripping

using namespace motor;
using namespace encoder;
// SET MOTOR DRIVER PIN PAIRS
const pin_pair MA_pins = motor2040::MOTOR_A;
const pin_pair MB_pins = motor2040::MOTOR_B;
const pin_pair MC_pins = motor2040::MOTOR_C;
const pin_pair MD_pins = motor2040::MOTOR_D;

// SET ENCODER PIN PAIRS
const pin_pair EA_pins = motor2040::ENCODER_A;
const pin_pair EB_pins = motor2040::ENCODER_B;
const pin_pair EC_pins = motor2040::ENCODER_C;
const pin_pair ED_pins = motor2040::ENCODER_D;

//SET MOTOR GEAR RATIOS
constexpr float GEAR_RATIO_A = 50.0f;
constexpr float GEAR_RATIO_B = 50.0f;
constexpr float GEAR_RATIO_C = 50.0f;
constexpr float GEAR_RATIO_D = 50.0f;

//SET MOTOR DEFAULT DIRECTIONS
const Direction DIRECTION_A = NORMAL_DIR;
const Direction DIRECTION_B = NORMAL_DIR;
const Direction DIRECTION_C = NORMAL_DIR;
const Direction DIRECTION_D = NORMAL_DIR;

// SET SPEED SCALING TO MATCH REAL WORLD SPEED
constexpr float SPEED_SCALE_A = 5.4f;
constexpr float SPEED_SCALE_B = 5.4f;
constexpr float SPEED_SCALE_C = 5.4f;
constexpr float SPEED_SCALE_D = 5.4f;

// SET PULSES PER REVOLUTION
constexpr float PULSE_A = 28.65f;
constexpr float PULSE_B = 28.65f;
constexpr float PULSE_C = 28.65f;
constexpr float PULSE_D = 28.65f;

// SET PID VALUES FOR EACH POSITION PDI OBJECT
constexpr float POS_KP_A = 0.025f;
constexpr float POS_KI_A = 0.0f;
constexpr float POS_KD_A = 0.0f;

constexpr float POS_KP_B = 0.025f;
constexpr float POS_KI_B = 0.0f;
constexpr float POS_KD_B = 0.0f;

constexpr float POS_KP_C = 0.025f;
constexpr float POS_KI_C = 0.0f;
constexpr float POS_KD_C = 0.0f;

constexpr float POS_KP_D = 0.025f;
constexpr float POS_KI_D = 0.0f;
constexpr float POS_KD_D = 0.0f;

// SET PID VALUES FOR EACH VELOCITY PID OBJECT
constexpr float VEL_KP_A = 30.0f;
constexpr float VEL_KI_A = 0.0f;
constexpr float VEL_KD_A = 0.4f;

constexpr float VEL_KP_B = 30.0f;
constexpr float VEL_KI_B = 0.0f;
constexpr float VEL_KD_B = 0.4f;

constexpr float VEL_KP_C = 30.0f;
constexpr float VEL_KI_C = 0.0f;
constexpr float VEL_KD_C = 0.4f;

constexpr float VEL_KP_D = 30.0f;
constexpr float VEL_KI_D = 0.0f;
constexpr float VEL_KD_D = 0.4f;

//SET UPDATE RATE FOR PID
const uint UPDATES = 100;
constexpr float UPDATE_RATE = 1.0f / (float)UPDATES;

//CREATE MOTOR OBJECTS
Motor m_a = Motor(MA_pins, DIRECTION_A, SPEED_SCALE_A);
Motor m_b = Motor(MB_pins, DIRECTION_B, SPEED_SCALE_B);
Motor m_c = Motor(MC_pins, DIRECTION_C, SPEED_SCALE_C);
Motor m_d = Motor(MD_pins, DIRECTION_D, SPEED_SCALE_D);

//CREATE ENCODER OBJECTS
Encoder enc_a = Encoder(pio0, 0, EA_pins, PIN_UNUSED, DIRECTION_A, PULSE_A, true);
Encoder enc_b = Encoder(pio0, 0, EB_pins, PIN_UNUSED, DIRECTION_B, PULSE_B, true);
Encoder enc_c = Encoder(pio0, 0, EC_pins, PIN_UNUSED, DIRECTION_C, PULSE_C, true);
Encoder enc_d = Encoder(pio0, 0, ED_pins, PIN_UNUSED, DIRECTION_D, PULSE_D, true);

//CREATE PID OBJECTS FOR VELOCITY
PID vel_pid_a = PID(VEL_KP_A, VEL_KI_A, VEL_KD_A, UPDATE_RATE);
PID vel_pid_b = PID(VEL_KP_B, VEL_KI_B, VEL_KD_B, UPDATE_RATE);
PID vel_pid_c = PID(VEL_KP_C, VEL_KI_C, VEL_KD_C, UPDATE_RATE);
PID vel_pid_d = PID(VEL_KP_D, VEL_KI_D, VEL_KD_D, UPDATE_RATE);

//CREATE PID OBJECTS FOR POSITION
PID pos_pid_a = PID(POS_KP_A, POS_KI_A, POS_KD_A, UPDATE_RATE);
PID pos_pid_b = PID(POS_KP_B, POS_KI_B, POS_KD_B, UPDATE_RATE);
PID pos_pid_c = PID(POS_KP_C, POS_KI_C, POS_KD_C, UPDATE_RATE);
PID pos_pid_d = PID(POS_KP_D, POS_KI_D, POS_KD_D, UPDATE_RATE);

//VARIABLES FOR USE THROUGHOUT MULTIPLE FUNCTIONS
char string[10]; // STRING TO STORE COMMUNICATION IN
int c; // INT TO STORE CHARACTER IN PRIOR TO STRING CONVERSION
Motor* MOTORS[4] = {&m_a, &m_b, &m_c, &m_d};
void init()
{
    //initialise usb serial communication
    stdio_init_all();
    //initialise motors and encoders
    m_a.init();
    m_b.init();
    m_c.init();
    m_d.init();

    enc_a.init();
    enc_b.init();
    enc_c.init();
    enc_d.init();
    
    //enable motors
    m_a.enable();
    m_b.enable();
    m_c.enable();
    m_d.enable();

}
void interpret_serial(int character) //TAKES THE CHARACTER INPUT FRO THE SERIAL INPUT AND CONVERTS TO A STRING
{
    int position = 0;
                
    do
    {
        string[position] = (char)character;
        position++;
        character = getchar_timeout_us(0);
    }while(character!='\n'&& character!= PICO_ERROR_TIMEOUT);
    string[position] = '\0';
    

}
void motor_control(char selection)
{
    Motor* motor = MOTORS[0];
    motor->full_positive();
}
int main()
{
    
    init();
    
    

    while (true)
    {
        c = getchar_timeout_us(0);
        if (c!=PICO_ERROR_TIMEOUT)
        {
            interpret_serial(c);
            if (string[0] == 'm')
            {
                printf("recieved\n");
                motor_control(string[1]);
            }
        }
        
    }
    return 0;
}