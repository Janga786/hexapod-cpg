// =============================================================================
//  config.h — Tuning parameters and pin map for hexapod_full
// =============================================================================
//
//  This header concentrates everything you might need to adjust on the field:
//  controller gains, bump-avoidance timing, kinematic geometry, and the full
//  Arduino-Mega pin assignment.  The .ino file pulls these symbols in via
//  `#include "config.h"` so the main control logic is uncluttered.
//
//  Units throughout: degrees for angles, milliseconds for time, millimetres
//  for lengths.  Counts ending in `_FRAMES` count 20 ms control frames
//  (the firmware runs at a fixed 50 Hz).
//
// =============================================================================

#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H

#include <Arduino.h>   // for the analog-pin constants A0..A5


// -----------------------------------------------------------------------------
//  Heading hold (yaw P-controller)
// -----------------------------------------------------------------------------

// Aggressiveness with which the controller drives the body back to the locked
// heading. Reasonable range: 1.0 – 5.0.  Increase if the robot ignores small
// drift; decrease if it zigzags.
constexpr float HEADING_KP    = 3.0f;

// Complementary-filter blend for fusing gyro + magnetometer.  A higher value
// trusts the gyro more (smoother, drifts slowly); lower trusts the
// magnetometer more (noisier, no drift).
constexpr float COMP_ALPHA    = 0.95f;


// -----------------------------------------------------------------------------
//  Bump-reaction state machine (forward walk only)
// -----------------------------------------------------------------------------

constexpr int  BUMP_BACKUP_FRAMES   = 40;     // 0.8 s back-up after a hit
constexpr int  BUMP_TURN_FRAMES     = 50;     // 1.0 s rotation in place
constexpr int  BUMP_RAMP_FRAMES     = 15;     // 0.3 s speed ramp on every transition
constexpr int  BACKUP_SPEED         = 100;    // 0–127 stride magnitude when reversing
constexpr int  TURN_SPEED           = 127;    // 0–127 rotation magnitude
constexpr unsigned long BUMP_DEBOUNCE_MS = 1500;  // ignore re-triggers for this long


// -----------------------------------------------------------------------------
//  Forward-walk parameters
// -----------------------------------------------------------------------------

constexpr int   WALK_SPEED          = 127;    // 0–127 stride magnitude when walking forward
constexpr float STEP_HEIGHT_MULT    = 2.0f;   // multiplier on swing-arc lift (0.5–2.0 sensible)


// -----------------------------------------------------------------------------
//  IMU
// -----------------------------------------------------------------------------

// I²C address-select pin on the SparkFun ICM-20948 breakout.
//   1 = AD0 high  → I²C address 0x69
//   0 = AD0 low   → I²C address 0x68
constexpr int IMU_AD0_VAL = 1;


// -----------------------------------------------------------------------------
//  Frame timing
// -----------------------------------------------------------------------------

constexpr int   FRAME_TIME_MS = 20;           // 20 ms = 50 Hz control loop
constexpr float DT_SECONDS    = 0.020f;       // ditto, expressed in seconds


// -----------------------------------------------------------------------------
//  Servo pin map (Arduino Mega 2560)
//
//  Each leg uses three contiguous pins for {coxa, femur, tibia}; legs are
//  numbered 1..6 starting from front-right and going counter-clockwise (i.e.
//  matching the leg-mount ordering used in the IK code).
// -----------------------------------------------------------------------------

constexpr int COXA1_SERVO  = 19;  constexpr int FEMUR1_SERVO = 8;   constexpr int TIBIA1_SERVO = 23;
constexpr int COXA2_SERVO  = 25;  constexpr int FEMUR2_SERVO = 27;  constexpr int TIBIA2_SERVO = 29;
constexpr int COXA3_SERVO  = 31;  constexpr int FEMUR3_SERVO = 33;  constexpr int TIBIA3_SERVO = 35;
constexpr int COXA4_SERVO  = 37;  constexpr int FEMUR4_SERVO = 39;  constexpr int TIBIA4_SERVO = 41;
constexpr int COXA5_SERVO  = 43;  constexpr int FEMUR5_SERVO = 45;  constexpr int TIBIA5_SERVO = 47;
constexpr int COXA6_SERVO  = 49;  constexpr int FEMUR6_SERVO = 51;  constexpr int TIBIA6_SERVO = 53;


// -----------------------------------------------------------------------------
//  3-bit binary command interface (latched on CONFIRM_PIN edge)
//
//  Truth table:
//    000  Idle (home pose)
//    001  Wave hello (front-right leg)
//    010  Forward walk + IMU heading hold + bump avoidance
//    011  Dance sway
//    100  Body pump (~3 Hz vertical bounce)
//    101  Twist (yaw rotation with double-frequency bounce)
//    110  Stomp (alternating tripod groups)
//    111  Full routine (cycles through 1, 4, 3, 5, 6, 4 every 3 s)
// -----------------------------------------------------------------------------

constexpr int CONFIRM_PIN = A2;
constexpr int BIT2_PIN    = A3;   // MSB (active low)
constexpr int BIT1_PIN    = A4;
constexpr int BIT0_PIN    = A5;   // LSB (active low)


// -----------------------------------------------------------------------------
//  Bump sensors (front foot pads)
// -----------------------------------------------------------------------------

constexpr int BUMP_LEFT_PIN  = 3;   // hit on left foot ⇒ rotate right after backup
constexpr int BUMP_RIGHT_PIN = 5;   // hit on right foot ⇒ rotate left after backup


// -----------------------------------------------------------------------------
//  Battery-monitor analog input
// -----------------------------------------------------------------------------

constexpr int BATT_VOLTAGE_PIN = 0;


// -----------------------------------------------------------------------------
//  LED bar-graph (8 dual-colour indicators)
// -----------------------------------------------------------------------------

constexpr int RED_LED1   = 22;  constexpr int GREEN_LED1 = 24;
constexpr int RED_LED2   = 26;  constexpr int GREEN_LED2 = 28;
constexpr int RED_LED3   = 30;  constexpr int GREEN_LED3 = 32;
constexpr int RED_LED4   = 34;  constexpr int GREEN_LED4 = 36;
constexpr int RED_LED5   = 38;  constexpr int GREEN_LED5 = 40;
constexpr int RED_LED6   = 42;  constexpr int GREEN_LED6 = 44;
constexpr int RED_LED7   = 46;  constexpr int GREEN_LED7 = 48;
constexpr int RED_LED8   = 50;  constexpr int GREEN_LED8 = 52;


// -----------------------------------------------------------------------------
//  Leg link lengths (mm) — must match the physical CAD
// -----------------------------------------------------------------------------

constexpr int COXA_LENGTH  = 51;
constexpr int FEMUR_LENGTH = 65;
constexpr int TIBIA_LENGTH = 121;
constexpr int TRAVEL       = 30;   // soft translate / rotate limit


// -----------------------------------------------------------------------------
//  Body geometry — neutral foot positions and coxa-mount offsets
//
//      LEG 1 = front-right
//      LEG 2 = mid-right
//      LEG 3 = back-right
//      LEG 4 = back-left
//      LEG 5 = mid-left
//      LEG 6 = front-left
// -----------------------------------------------------------------------------

constexpr float HOME_X[6] = {  82.0f,    0.0f,  -82.0f,   -82.0f,    0.0f,   82.0f };
constexpr float HOME_Y[6] = {  82.0f,  116.0f,   82.0f,   -82.0f, -116.0f,  -82.0f };
constexpr float HOME_Z[6] = { -80.0f,  -80.0f,  -80.0f,   -80.0f,  -80.0f,  -80.0f };

constexpr float BODY_X[6] = { 110.4f,   0.0f, -110.4f,  -110.4f,    0.0f,  110.4f };
constexpr float BODY_Y[6] = {  58.4f,  90.8f,   58.4f,   -58.4f,  -90.8f,  -58.4f };
constexpr float BODY_Z[6] = {   0.0f,   0.0f,    0.0f,     0.0f,    0.0f,    0.0f };


// -----------------------------------------------------------------------------
//  Per-axis servo calibration (degrees)
//
//  These are the per-leg corrections applied after the IK math.  They absorb
//  small mechanical assembly tolerances and servo-arm-mounting offsets.
//  Re-tune by running the legacy_ps2 sketch in "set all servos to 90°"
//  calibration mode and physically straightening each link, then nudging
//  these values until the foot tip lands on the home position.
// -----------------------------------------------------------------------------

constexpr int COXA_CAL[6]  = {  2, -1, -1, -3, -2, -3 };
constexpr int FEMUR_CAL[6] = {  4, -2,  0, -1,  0,  0 };
constexpr int TIBIA_CAL[6] = {  0, -3, -3, -2, -3, -1 };


#endif  // HEXAPOD_CONFIG_H
