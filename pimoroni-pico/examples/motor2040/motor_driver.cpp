
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
constexpr float POS_KD_C = 0.0f

constexpr float POS_KP_D = 0.025f;
constexpr float POS_KI_D = 0.0f;
constexpr float POS_KD_D = 0.0f;

//CREATE MOTOR OBJECTS
Motor m_a = MOTOR(MA_pins, DIRECTION_A, SPEED_SCALE_A);
Motor m_b = MOTOR(MB_pins, DIRECTION_B, SPEED_SCALE_B);
Motor m_c = MOTOR(MC_pins, DIRECTION_C, SPEED_SCALE_C);
Motor m_d = MOTOR(MD_pins, DIRECTION_D, SPEED_SCALE_D);

//CREATE ENCODER OBJECTS
ENCODER enc_a = ENCODER(pio0, 0, EA_pins, PIN_UNUSED, DIRECTION_A, PULSE_A, true);
ENCODER enc_b = ENCODER(pio0, 0, EB_pins, PIN_UNUSED, DIRECTION_B, PULSE_B, true);
ENCODER enc_c = ENCODER(pio0, 0, EC_pins, PIN_UNUSED, DIRECTION_C, PULSE_C, true);
ENCODER enc_d = ENCODER(pio0, 0, ED_pins, PIN_UNUSED, DIRECTION_D, PULSE_D, true);
