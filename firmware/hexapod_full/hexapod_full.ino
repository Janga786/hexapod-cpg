//***********************************************************************
// Hexapod Program - Obstacle Course + Dance Performance
// Code for Arduino Mega
//
// Truth Table:
//   000 = Idle (home position)
//   001 = Wave (front-right leg waves hello)
//   010 = Forward walk with IMU heading hold + bump obstacle avoidance
//   011 = Dance Sway (rock side-to-side with swagger)
//   100 = Body Pump (fast ~3Hz up-down bounce)
//   101 = Twist (body yaw rotation with double-freq bounce)
//   110 = Stomp (alternating tripod groups stomp)
//   111 = Full Routine (cycles through all 6 actions, 3 sec each)
//
// by Mark W (original), modified by Jangara
//***********************************************************************

#include <Servo.h>
#include <Wire.h>
#include <ICM_20948.h>   // SparkFun ICM-20948 library
#include <math.h>

#include "config.h"      // pin map, controller gains, geometry, calibration

// Convenience aliases mapped onto config.h symbols
constexpr int   BATT_VOLTAGE = BATT_VOLTAGE_PIN;
constexpr float DT           = DT_SECONDS;


//***********************************************************************
// Variable Declarations
//***********************************************************************
unsigned long currentTime;
unsigned long previousTime;

// Mode variable
//   mode 0   = idle / home
//   mode 1   = forward walk with heading hold (command 010)
//   mode 10  = bump reaction: backing up
//   mode 11  = bump reaction: turning
//   mode 20  = dance/performance action
int mode;
int gait_speed;
int reset_position;

// Binary input variables
int binary_command;
bool confirm_prev = HIGH;

// Bump sensor variables
bool bump_left_prev  = HIGH;
bool bump_right_prev = HIGH;
int bump_turn_dir    = 0;
int bump_frame_count = 0;
unsigned long last_bump_time = 0;
int forward_ramp_count = 0;

// IMU heading variables
float current_heading  = 0.0;
float desired_heading  = 0.0;
float heading_error    = 0.0;
float gyro_bias_z      = 0.0;   // auto-calibrated at startup
bool  heading_locked   = false;
bool  imu_ready        = false;

// Dance/performance variables
int perf_tick = 0;                   // frame counter for dance animations
int perf_action = 0;                 // which dance action is active (1-7)

// Battery monitor variables
int batt_LEDs;
int batt_voltage;
int batt_voltage_index;
int batt_voltage_array[50];
long batt_voltage_sum;

// IK variables
float L0, L3;
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

// Walking variables
int leg_num;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int commandedX, commandedY, commandedR;
float strideX, strideY, strideR;
float sinRotZ, cosRotZ;
float rotOffsetX, rotOffsetY;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6] = {1,2,1,2,1,2};


//***********************************************************************
// Object Declarations
//***********************************************************************
ICM_20948_I2C myICM;

Servo coxa1_servo;
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;


//***********************************************************************
// Helper: wrap angle to -180 to +180
//***********************************************************************
float wrap_angle(float angle)
{
  while(angle >  180.0) angle -= 360.0;
  while(angle < -180.0) angle += 360.0;
  return angle;
}


//***********************************************************************
// Helper: reset tripod gait state
//***********************************************************************
void reset_tripod_cases()
{
  for(int i = 0; i < 6; i++)
    tripod_case[i] = (i % 2 == 0) ? 1 : 2;
  tick = 0;
}


//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  Serial.begin(115200);

  //--- Input pins ---
  pinMode(CONFIRM_PIN, INPUT_PULLUP);
  pinMode(BIT2_PIN, INPUT_PULLUP);
  pinMode(BIT1_PIN, INPUT_PULLUP);
  pinMode(BIT0_PIN, INPUT_PULLUP);
  pinMode(BUMP_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUMP_RIGHT_PIN, INPUT_PULLUP);

  //--- IMU setup ---
  Wire.begin();
  Wire.setClock(400000);

  myICM.begin(Wire, IMU_AD0_VAL);
  if(myICM.status == ICM_20948_Stat_Ok)
  {
    imu_ready = true;
    Serial.println("ICM-20948 connected");
  }
  else
  {
    imu_ready = false;
    Serial.print("ICM-20948 FAILED, status: ");
    Serial.println(myICM.statusString());
    Serial.println("Walking mode will run without heading hold");
  }

  if(imu_ready)
  {
    delay(100);

    // prime heading with magnetometer
    for(int i = 0; i < 20; i++)
    {
      if(myICM.dataReady())
      {
        myICM.getAGMT();
        current_heading = atan2(myICM.magY(), myICM.magX()) * RAD_TO_DEG;
      }
      delay(10);
    }
    Serial.print("Initial heading: ");
    Serial.println(current_heading);

    // ---- GYRO BIAS CALIBRATION ----
    // Robot MUST be sitting still during this!
    // Samples gyro Z for 1 second to find the resting offset
    Serial.println("Calibrating gyro bias... KEEP ROBOT STILL!");
    float bias_sum = 0.0;
    int bias_count = 0;
    for(int i = 0; i < 50; i++)   // 50 samples over 1 second
    {
      if(myICM.dataReady())
      {
        myICM.getAGMT();
        bias_sum += myICM.gyrZ();
        bias_count++;
      }
      delay(20);
    }
    if(bias_count > 0)
      gyro_bias_z = bias_sum / (float)bias_count;
    else
      gyro_bias_z = 0.0;

    Serial.print("Gyro Z bias: ");
    Serial.print(gyro_bias_z, 4);
    Serial.println(" deg/s");
    Serial.println("Bias calibration done.");
  }

  //--- Servo setup ---
  coxa1_servo.attach(COXA1_SERVO,610,2400);
  femur1_servo.attach(FEMUR1_SERVO,610,2400);
  tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);
  femur2_servo.attach(FEMUR2_SERVO,610,2400);
  tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);
  femur3_servo.attach(FEMUR3_SERVO,610,2400);
  tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);
  femur4_servo.attach(FEMUR4_SERVO,610,2400);
  tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);
  femur5_servo.attach(FEMUR5_SERVO,610,2400);
  tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);
  femur6_servo.attach(FEMUR6_SERVO,610,2400);
  tibia6_servo.attach(TIBIA6_SERVO,610,2400);

  //--- LED setup ---
  for(int i = 0; i < 8; i++)
  {
    pinMode((RED_LED1 + (4*i)), OUTPUT);
    pinMode((GREEN_LED1 + (4*i)), OUTPUT);
  }

  //--- Battery monitor setup ---
  for(batt_voltage_index = 0; batt_voltage_index < 50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum = 0;
  batt_voltage_index = 0;

  //--- Walking state setup ---
  for(leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }

  mode = 0;
  gait_speed = 0;
  reset_position = true;
  binary_command = 0;
  heading_locked = false;
  perf_tick = 0;
  perf_action = 0;
}


//***********************************************************************
// Main Program Loop
//***********************************************************************
void loop()
{
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    // update IMU heading every frame
    update_heading();

    // read binary input commands
    process_binary_input();

    // check bump sensors (only relevant during walk mode)
    process_bump_sensors();

    // reset legs to home position when commanded
    if(reset_position == true)
    {
      for(leg_num = 0; leg_num < 6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    // compute IK and write servos
    for(leg_num = 0; leg_num < 6; leg_num++)
      leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num],
                      current_Y[leg_num] + offset_Y[leg_num],
                      current_Z[leg_num] + offset_Z[leg_num]);

    // housekeeping
    battery_monitor();
    print_debug();

    //--- Process active mode ---
    if(mode == 1)   tripod_gait_heading_hold();   // 010: forward walk
    if(mode == 10)  tripod_gait_backup();          // bump: backing up
    if(mode == 11)  tripod_gait_turn();            // bump: turning
    if(mode == 20)  perform_action();              // dance/performance
  }
}


//***********************************************************************
// Update IMU Heading (complementary filter)
//***********************************************************************
void update_heading()
{
  if(!imu_ready) return;
  if(!myICM.dataReady()) return;

  myICM.getAGMT();

  float gz = myICM.gyrZ() - gyro_bias_z;  // subtract resting bias
  float mag_heading = atan2(myICM.magY(), myICM.magX()) * RAD_TO_DEG;

  float heading_pred = current_heading + gz * DT;
  float mag_error = wrap_angle(mag_heading - heading_pred);
  current_heading = wrap_angle(heading_pred + (1.0 - COMP_ALPHA) * mag_error);
}


//***********************************************************************
// Process binary input
//
// Truth Table:
//   000 (0) = Idle
//   001 (1) = Wave
//   010 (2) = Forward walk with heading hold + bump avoidance
//   011 (3) = Dance Sway
//   100 (4) = Body Pump
//   101 (5) = Twist
//   110 (6) = Stomp
//   111 (7) = Full Routine
//***********************************************************************
void process_binary_input()
{
  bool confirm_now = digitalRead(CONFIRM_PIN);

  if(confirm_prev == HIGH && confirm_now == LOW)
  {
    int bit2 = !digitalRead(BIT2_PIN);
    int bit1 = !digitalRead(BIT1_PIN);
    int bit0 = !digitalRead(BIT0_PIN);
    binary_command = (bit2 << 2) | (bit1 << 1) | bit0;

    Serial.print("Binary command: ");
    Serial.println(binary_command);

    // reset common state
    reset_position = true;
    perf_tick = 0;

    switch(binary_command)
    {
      case 0:  // 000 = idle
        mode = 0;
        heading_locked = false;
        perf_action = 0;
        break;

      case 2:  // 010 = walk forward with heading hold
        mode = 1;
        reset_tripod_cases();
        forward_ramp_count = 0;
        desired_heading = current_heading;
        heading_locked = true;
        perf_action = 0;
        Serial.print("Heading locked at: ");
        Serial.println(desired_heading);
        break;

      case 1:  // 001 = wave
      case 3:  // 011 = dance sway
      case 4:  // 100 = body pump
      case 5:  // 101 = twist
      case 6:  // 110 = stomp
      case 7:  // 111 = full routine
        mode = 20;
        perf_action = binary_command;
        heading_locked = false;
        break;
    }
  }

  confirm_prev = confirm_now;
}


//***********************************************************************
// Process bump sensors
//***********************************************************************
void process_bump_sensors()
{
  bool bump_left_now  = digitalRead(BUMP_LEFT_PIN);
  bool bump_right_now = digitalRead(BUMP_RIGHT_PIN);

  if(mode == 1 && (millis() - last_bump_time) > BUMP_DEBOUNCE_MS)
  {
    if(bump_left_prev == HIGH && bump_left_now == LOW)
    {
      Serial.println("BUMP LEFT -> backup + turn right");
      mode = 10;
      bump_turn_dir = 1;
      bump_frame_count = 0;
      last_bump_time = millis();
      reset_position = true;
      reset_tripod_cases();
    }
    else if(bump_right_prev == HIGH && bump_right_now == LOW)
    {
      Serial.println("BUMP RIGHT -> backup + turn left");
      mode = 10;
      bump_turn_dir = -1;
      bump_frame_count = 0;
      last_bump_time = millis();
      reset_position = true;
      reset_tripod_cases();
    }
  }

  bump_left_prev  = bump_left_now;
  bump_right_prev = bump_right_now;
}


//***********************************************************************
// Tripod Gait - Forward Walk with Heading Hold
//***********************************************************************
void tripod_gait_heading_hold()
{
  float ramp;
  if(forward_ramp_count < BUMP_RAMP_FRAMES)
  {
    ramp = (float)(forward_ramp_count + 1) / (float)BUMP_RAMP_FRAMES;
    forward_ramp_count++;
  }
  else
    ramp = 1.0;

  commandedX = (int)(WALK_SPEED * ramp);
  commandedY = 0;

  if(heading_locked && imu_ready)
  {
    heading_error = wrap_angle(desired_heading - current_heading);
    commandedR = constrain((int)(HEADING_KP * heading_error), -127, 127);
  }
  else
    commandedR = 0;

  compute_strides();
  numTicks = round(duration / FRAME_TIME_MS / 2.0);
  for(leg_num = 0; leg_num < 6; leg_num++)
  {
    compute_amplitudes();
    switch(tripod_case[leg_num])
    {
      case 1:
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
        if(tick >= numTicks-1) tripod_case[leg_num] = 2;
        break;
      case 2:
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        if(tick >= numTicks-1) tripod_case[leg_num] = 1;
        break;
    }
  }
  if(tick < numTicks-1) tick++;
  else tick = 0;
}


//***********************************************************************
// Tripod Gait - Backup (after bump)
//***********************************************************************
void tripod_gait_backup()
{
  float ramp;
  if(bump_frame_count < BUMP_RAMP_FRAMES)
    ramp = (float)(bump_frame_count + 1) / (float)BUMP_RAMP_FRAMES;
  else
    ramp = 1.0;

  commandedX = -(int)(BACKUP_SPEED * ramp);
  commandedY = 0;
  commandedR = 0;

  compute_strides();
  numTicks = round(duration / FRAME_TIME_MS / 2.0);
  for(leg_num = 0; leg_num < 6; leg_num++)
  {
    compute_amplitudes();
    switch(tripod_case[leg_num])
    {
      case 1:
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
        if(tick >= numTicks-1) tripod_case[leg_num] = 2;
        break;
      case 2:
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        if(tick >= numTicks-1) tripod_case[leg_num] = 1;
        break;
    }
  }
  if(tick < numTicks-1) tick++;
  else tick = 0;

  bump_frame_count++;
  if(bump_frame_count >= BUMP_BACKUP_FRAMES)
  {
    Serial.println("Backup done -> turning");
    mode = 11;
    bump_frame_count = 0;
    reset_position = true;
    reset_tripod_cases();
  }
}


//***********************************************************************
// Tripod Gait - Turn (after backup)
//***********************************************************************
void tripod_gait_turn()
{
  commandedX = 0;
  commandedY = 0;
  commandedR = TURN_SPEED * bump_turn_dir;

  compute_strides();
  numTicks = round(duration / FRAME_TIME_MS / 2.0);
  for(leg_num = 0; leg_num < 6; leg_num++)
  {
    compute_amplitudes();
    switch(tripod_case[leg_num])
    {
      case 1:
        current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
        if(tick >= numTicks-1) tripod_case[leg_num] = 2;
        break;
      case 2:
        current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
        current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
        current_Z[leg_num] = HOME_Z[leg_num];
        if(tick >= numTicks-1) tripod_case[leg_num] = 1;
        break;
    }
  }
  if(tick < numTicks-1) tick++;
  else tick = 0;

  bump_frame_count++;
  if(bump_frame_count >= BUMP_TURN_FRAMES)
  {
    Serial.print("Turn done -> resuming forward, correction: ");
    Serial.println(wrap_angle(desired_heading - current_heading));
    mode = 1;
    bump_frame_count = 0;
    forward_ramp_count = 0;
    reset_position = true;
    reset_tripod_cases();
  }
}


//***********************************************************************
// Performance Action Dispatcher
//***********************************************************************
void perform_action()
{
  switch(perf_action)
  {
    case 1: action_wave();       break;   // 001
    case 3: action_dance_sway(); break;   // 011
    case 4: action_body_pump();  break;   // 100
    case 5: action_twist();      break;   // 101
    case 6: action_stomp();      break;   // 110
    case 7: action_routine();    break;   // 111
  }
  perf_tick++;
}


//***********************************************************************
// Action 001: Wave - front right leg waves hello
//***********************************************************************
void action_wave()
{
  // keep legs 2-6 at home
  for(int i = 1; i < 6; i++)
  {
    current_X[i] = HOME_X[i];
    current_Y[i] = HOME_Y[i];
    current_Z[i] = HOME_Z[i];
  }

  if(perf_tick < 20)                          // raise leg 1 over 0.4 sec
  {
    float t = (float)perf_tick / 20.0;
    current_X[0] = HOME_X[0];
    current_Y[0] = HOME_Y[0] - 30.0 * t;    // pull in slightly
    current_Z[0] = HOME_Z[0] + 120.0 * t;   // raise up high
  }
  else                                        // wave back and forth
  {
    int wt = (perf_tick - 20) % 40;
    current_X[0] = HOME_X[0] + 50.0 * sin(2.0 * M_PI * wt / 40.0);
    current_Y[0] = HOME_Y[0] - 30.0;
    current_Z[0] = HOME_Z[0] + 120.0;
  }
}


//***********************************************************************
// Action 100: Body Pump - fast ~3Hz up-down bounce
//***********************************************************************
void action_body_pump()
{
  float z_offset = 30.0 * sin(2.0 * M_PI * perf_tick / 16.0);
  for(int i = 0; i < 6; i++)
  {
    current_X[i] = HOME_X[i];
    current_Y[i] = HOME_Y[i];
    current_Z[i] = HOME_Z[i] + z_offset;
  }
}


//***********************************************************************
// Action 011: Dance Sway - rock body side to side with swagger
//***********************************************************************
void action_dance_sway()
{
  float phase = 2.0 * M_PI * perf_tick / 40.0;   // ~1.25Hz rock
  float x_sway = 15.0 * sin(phase * 0.5);         // slower forward-back swagger

  for(int i = 0; i < 6; i++)
  {
    current_X[i] = HOME_X[i] + x_sway;
    current_Y[i] = HOME_Y[i];
    if(i < 3)                                      // right side legs up
      current_Z[i] = HOME_Z[i] + 25.0 * sin(phase);
    else                                           // left side legs down
      current_Z[i] = HOME_Z[i] - 25.0 * sin(phase);
  }
}


//***********************************************************************
// Action 101: Twist - body yaw rotation with double-freq bounce
//***********************************************************************
void action_twist()
{
  float angle = 25.0 * sin(2.0 * M_PI * perf_tick / 50.0);   // yaw sweep
  float sinR = sin(radians(angle));
  float cosR = cos(radians(angle));
  float z_bounce = 10.0 * sin(2.0 * M_PI * perf_tick / 25.0); // double-freq bounce

  for(int i = 0; i < 6; i++)
  {
    float tX = HOME_X[i] + BODY_X[i];
    float tY = HOME_Y[i] + BODY_Y[i];
    float rotX = tY * sinR + tX * cosR - tX;
    float rotY = tY * cosR - tX * sinR - tY;
    current_X[i] = HOME_X[i] + rotX;
    current_Y[i] = HOME_Y[i] + rotY;
    current_Z[i] = HOME_Z[i] + z_bounce;
  }
}


//***********************************************************************
// Action 110: Stomp - alternating tripod groups stomp
//***********************************************************************
void action_stomp()
{
  int stomp_period = 24;                        // frames per stomp cycle
  int st = perf_tick % stomp_period;

  for(int i = 0; i < 6; i++)
  {
    current_X[i] = HOME_X[i];
    current_Y[i] = HOME_Y[i];

    bool group_a = (i == 0 || i == 2 || i == 4);
    float z_mod = 0;

    if(group_a)
    {
      if(st < stomp_period / 2)
        z_mod = 40.0 * sin(M_PI * st / (stomp_period / 2.0));
    }
    else
    {
      if(st >= stomp_period / 2)
        z_mod = 40.0 * sin(M_PI * (st - stomp_period / 2) / (stomp_period / 2.0));
    }
    current_Z[i] = HOME_Z[i] + z_mod;
  }
}


//***********************************************************************
// Action 111: Full Routine - cycles through all 6 actions, 3 sec each
//***********************************************************************
void action_routine()
{
  int action_duration = 150;                      // 3 sec per action at 50Hz
  int total_cycle = action_duration * 6;
  int total_tick = perf_tick % total_cycle;
  int current_action = total_tick / action_duration;
  int sub_tick = total_tick % action_duration;

  int saved_tick = perf_tick;
  perf_tick = sub_tick;

  switch(current_action)
  {
    case 0: action_wave();       break;
    case 1: action_body_pump();  break;
    case 2: action_dance_sway(); break;
    case 3: action_twist();      break;
    case 4: action_stomp();      break;
    case 5: action_body_pump();  break;   // repeat pump as finale
  }

  perf_tick = saved_tick;
}


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  if((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2*FEMUR_LENGTH*TIBIA_LENGTH));
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2*FEMUR_LENGTH*L3));
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    theta_coxa = atan2(X, Y)*RAD_TO_DEG + COXA_CAL[leg_number];

    switch(leg_number)
    {
      case 0:
        theta_coxa = theta_coxa + 45.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa1_servo.write(int(theta_coxa));
        femur1_servo.write(int(theta_femur));
        tibia1_servo.write(int(theta_tibia));
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa2_servo.write(int(theta_coxa));
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;
      case 2:
        theta_coxa = theta_coxa + 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa3_servo.write(int(theta_coxa));
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;
      case 3:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 225.0;
        else
          theta_coxa = theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa4_servo.write(int(theta_coxa));
        femur4_servo.write(int(theta_femur));
        tibia4_servo.write(int(theta_tibia));
        break;
      case 4:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 270.0;
        else
          theta_coxa = theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa5_servo.write(int(theta_coxa));
        femur5_servo.write(int(theta_femur));
        tibia5_servo.write(int(theta_tibia));
        break;
      case 5:
        if(theta_coxa < 0)
          theta_coxa = theta_coxa + 315.0;
        else
          theta_coxa = theta_coxa - 45.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa6_servo.write(int(theta_coxa));
        femur6_servo.write(int(theta_femur));
        tibia6_servo.write(int(theta_tibia));
        break;
    }
  }
}


//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  strideX = 90 * commandedX / 127;
  strideY = 90 * commandedY / 127;
  strideR = 35 * commandedR / 127;

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));

  if(gait_speed == 0) duration = 1080;
  else duration = 3240;
}


//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX;
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY;

  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50);
  amplitudeY = constrain(amplitudeY, -50, 50);

  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = STEP_HEIGHT_MULT * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = STEP_HEIGHT_MULT * (strideY + rotOffsetY) / 4.0;
}


//***********************************************************************
// Battery monitor routine
//***********************************************************************
void battery_monitor()
{
  batt_voltage_sum = batt_voltage_sum - batt_voltage_array[batt_voltage_index];
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE), 0, 1023, 0, 1497);
  batt_voltage_sum = batt_voltage_sum + batt_voltage_array[batt_voltage_index];
  batt_voltage_index = batt_voltage_index + 1;
  if(batt_voltage_index > 49) batt_voltage_index = 0;

  batt_voltage = batt_voltage_sum / 50;
  batt_LEDs = map(constrain(batt_voltage, 1020, 1230), 1020, 1230, 1, 8);
  if(batt_LEDs > 3) LED_Bar(1, batt_LEDs);
  else LED_Bar(0, batt_LEDs);
}


//***********************************************************************
// LED Bar Graph Routine
//***********************************************************************
void LED_Bar(int LED_color, int LED_count)
{
  if(LED_color == 0)
  {
    for(int i = 0; i < LED_count; i++)
    {
      digitalWrite((RED_LED1 + (4*i)), HIGH);
      digitalWrite((GREEN_LED1 + (4*i)), LOW);
    }
    for(int i = LED_count; i < 8; i++)
    {
      digitalWrite((RED_LED1 + (4*i)), LOW);
      digitalWrite((GREEN_LED1 + (4*i)), LOW);
    }
  }
  else
  {
    for(int i = 0; i < LED_count; i++)
    {
      digitalWrite((GREEN_LED1 + (4*i)), HIGH);
      digitalWrite((RED_LED1 + (4*i)), LOW);
    }
    for(int i = LED_count; i < 8; i++)
    {
      digitalWrite((GREEN_LED1 + (4*i)), LOW);
      digitalWrite((RED_LED1 + (4*i)), LOW);
    }
  }
}


//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  currentTime = millis();
  Serial.print(currentTime - previousTime);
  Serial.print(",V=");
  Serial.print(float(batt_voltage) / 100.0);
  Serial.print(",mode=");
  Serial.print(mode);
  Serial.print(",act=");
  Serial.print(perf_action);
  Serial.print(",hdg=");
  Serial.print(current_heading, 1);
  Serial.print(",err=");
  Serial.print(heading_error, 1);
  Serial.println();
}
