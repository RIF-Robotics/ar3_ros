/*
  Arduino Sketch for AR3 with ROS2 Support
  Author: Kevin DeMarco <kevin@kevindemarco.com>
  Date: 2022
*/

const char version[] = "0.0.1";
const int NUM_JOINTS = 6;

// SPEED // millisecond multiplier // raise value to slow robot speeds // DEFAULT = 220
const int SpeedMult = 220;

/*
  MOTOR DIRECTION - motor directions can be changed on the caibration page in the software but can also
  be changed here: example using DM542T driver(CW) set to 1 - if using ST6600 or DM320T driver(CCW) set to 0
  DEFAULT = 111011   */

const int J1rotdir = 1;
const int J2rotdir = 1;
const int J3rotdir = 1;
const int J4rotdir = 0;
const int J5rotdir = 1;
const int J6rotdir = 1;
const int TRACKrotdir = 0;
const int rot_dirs[] = {J1rotdir, J2rotdir, J3rotdir, J4rotdir, J5rotdir, J6rotdir};
int rot_dir_sign[NUM_JOINTS];

/* start positions - these are the joint step values at power up, default is in the rest position using
   the following values: J1=7600, J2=2322, J3=0, J4=7600, J5=2287, J6=3312 */

int J1startSteps = 7600;
int J2startSteps = 2322;
int J3startSteps = 0;
int J4startSteps = 7600;
int J5startSteps = 2287;
int J6startSteps = 3312;
const int start_steps[] = {J1startSteps, J2startSteps, J3startSteps, J4startSteps, J5startSteps, J6startSteps};

// #define ENCODER_OPTIMIZE_INTERRUPTS # TODO, try this out later
// https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <Encoder.h>
#include <avr/pgmspace.h>

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int TRstepPin = 12;
const int TRdirPin = 13;

const int step_pins[] = {J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin};
const int dir_pins[] = {J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin};

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(16, 17);
Encoder J3encPos(18, 19);
Encoder J4encPos(20, 21);
Encoder J5encPos(22, 23);
Encoder J6encPos(24, 25);
Encoder* encoders[] = {&J1encPos, &J2encPos, &J3encPos, &J4encPos, &J5encPos, &J6encPos};

float desired_joint_positions_rad[NUM_JOINTS];
float current_joint_positions_rad[NUM_JOINTS];
float joint_positions_err_rad[NUM_JOINTS];

float current_joint_positions_from_steps_rad[NUM_JOINTS];

int current_encoder_counts[NUM_JOINTS];
int desired_encoder_counts[NUM_JOINTS];
int encoder_count_err[NUM_JOINTS];

int current_motor_steps[] = {0, 0, 0, 0, 0, 0};
int desired_motor_steps[NUM_JOINTS];
int desired_motor_dirs[] = {0, 0, 0, 0, 0, 0};
int prev_desired_motor_dirs[] = {0, 0, 0, 0, 0, 0};

//set calibration limit switch pins
const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int limit_switches[] = {J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin};
const unsigned int cal_dirs[] = {0, 0, 1, 0, 0, 1};

int limit_switch_states[] = {0, 0, 0, 0, 0, 0};
int last_limit_switch_states[] = {0, 0, 0, 0, 0, 0};
unsigned long last_debounce_times[] = {0, 0, 0, 0, 0, 0};
unsigned long debounce_delay = 10000;
int limit_switch_rising_edges[] = {0, 0, 0, 0, 0, 0};

//set encoder multiplier
const float J1encMult = 5.12;
const float J2encMult = 5.12;
const float J3encMult = 5.12;
const float J4encMult = 5.12;
const float J5encMult = 2.56;
const float J6encMult = 5.12;
const float EncDiv = .1;
const float encoder_mults[] = {J1encMult, J2encMult, J3encMult, J4encMult, J5encMult, J6encMult};

unsigned long time = 0;
unsigned long prev_print_time[] = {0, 0, 0, 0, 0, 0};
unsigned long time_unit = 1000000;
int one_second = 1 * time_unit;
int half_second = time_unit / 2.0;

const float joint_neg_limits_rad[] = {-170.0 * M_PI/180.0, -129.6 * M_PI/180.0,  +1.0 * M_PI/180.0, -164.5 * M_PI/180.0, -104.15 * M_PI/180.0, -148.1 * M_PI/180.0};
const float joint_pos_limits_rad[] = {+170.0 * M_PI/180.0,   +0.0 * M_PI/180.0, 143.7 * M_PI/180.0,  164.5 * M_PI/180.0, +104.15 * M_PI/180.0, +148.1 * M_PI/180.0};
const float enc_dir[] = {-1, +1, +1, +1, +1, +1};
const int  motor_step_limits[] = { 15110,   7198,  7984,  14056,    4560,   6320};
int joint_pos_limit_encoder_count[NUM_JOINTS];
int joint_pos_limit_motor_steps[NUM_JOINTS];

int encoder_count_limits[NUM_JOINTS];
int motor_steps_per_rad[NUM_JOINTS];
int encoder_counts_per_rad[NUM_JOINTS];

bool enable_debug[] = {false, false, false, false, false, false};
bool enable_control_loop[] = {false, false, false, false, false, false};

/// State Machine Variables
enum state {INACTIVE, SET_DIRECTION, SET_DIRECTION_DELAY, TOGGLE_STEP_LOW, TOGGLE_STEP_LOW_DELAY, TOGGLE_STEP_HIGH, TOGGLE_STEP_HIGH_DELAY};
state current_state[] = {INACTIVE, INACTIVE, INACTIVE, INACTIVE, INACTIVE, INACTIVE};
state next_state[] = {INACTIVE, INACTIVE, INACTIVE, INACTIVE, INACTIVE, INACTIVE};
unsigned long delay_until[] = {0, 0, 0, 0, 0, 0};

// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 100;

////////////////////////////////////////////////////////////////////////////////
/// Helper Functions
////////////////////////////////////////////////////////////////////////////////
template <typename T>
Print& operator<<(Print& printer, T value)
{
  printer.print(value);
  return printer;
}

void read_encoder_counts(int* encoder_positions)
{
  // This function takes between 1 and 2 microseconds to complete
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    encoder_positions[i] = encoders[i]->read();
  }
}

void write_encoder_counts(const int* encoder_positions)
{
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    encoders[i]->write(encoder_positions[i] * encoder_mults[i]);
  }
}

template <typename T>
void array_to_message(T* array, unsigned int length, int precision, String* msg)
{
  for (unsigned int i = 0 ; i < length; ++i) {
    *msg += String(array[i], precision) + ",";
  }
}

template <typename T>
void array_to_message(T* array, unsigned int length, String* msg)
{
  for (unsigned int i = 0 ; i < length; ++i) {
    *msg += String(array[i]) + ",";
  }
}

template <typename T>
int sgn(T value) {
  return (T(0) < value) - (value < T(0));
}

bool parse_line(char* line, const char* delim, unsigned int expected_length, String* tokens)
{
  char* token = strtok(line, delim);
  unsigned int i = 0;
  while (token != NULL && i < expected_length) {
    tokens[i] = String(token);
    token = strtok(NULL, delim);
    ++i;
  }
  return i == expected_length && token == NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// setup() function
////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  // Set encoders to known values
  write_encoder_counts(start_steps);

  // Set pin modes
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(step_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(limit_switches[i], INPUT_PULLUP);
    digitalWrite(step_pins[i], HIGH);
    digitalWrite(dir_pins[i], HIGH);
  }

  // Compute steps per degree
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    encoder_count_limits[i] = motor_step_limits[i] * encoder_mults[i];
    motor_steps_per_rad[i] = round((float)motor_step_limits[i] / (joint_pos_limits_rad[i] - joint_neg_limits_rad[i]));
    encoder_counts_per_rad[i] = round((float)motor_step_limits[i] / (joint_pos_limits_rad[i] - joint_neg_limits_rad[i]) * encoder_mults[i]);

    if (enc_dir[i] > 0) {
      joint_pos_limit_encoder_count[i] = encoder_count_limits[i];
      joint_pos_limit_motor_steps[i] = motor_step_limits[i];
    } else {
      joint_pos_limit_encoder_count[i] = 0;
      joint_pos_limit_motor_steps[i] = 0;
    }

    rot_dir_sign[i] = rot_dirs[i] == 1 ? +1 : -1;
  }

} // setup()

////////////////////////////////////////////////////////////////////////////////
/// Main loop
////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // TODO: Handle clock rollover
  time = micros();

  update_limit_switch_states();

  // Update encoder counts and joint angles once per loop
  calculate_current_joint_positions();

  // Handle any incoming serial messages
  process_incoming_serial();

  // Run the motor control loop
  control_loop_motor_steps();

  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    if (enable_debug[i]) {
      print_debug_loop(half_second, i);
    }
  }
}  // end of loop

////////////////////////////////////////////////////////////////////////////////
/// Process incoming command messages over serial
////////////////////////////////////////////////////////////////////////////////
void process_incoming_message(char * data)
{
  switch (data[0]) {
    case 'E':
      send_encoder_counts();
      break;
    case 'P':
      send_joint_positions();
      break;
    case 'D':
      set_desired_joint_positions(data);
      break;
    case 'C':
      calibrate_encoders(data);
      break;
    case 'V':
      Serial.println(version);
      break;
    case 'L':
      drive_to_limit_switches(data);
      break;
    case 'M':
      old_move(data);
      break;
    case 'B':
      handle_enable(String("b"),data, enable_debug, String("debug"));
      break;
    case 'X':
      handle_enable(String("x"),data, enable_control_loop, String("control loop"));
      break;
    case 'S':
      manual_steps(data);
      break;
    case 'Q':
      send_error_status();
      break;
    case 'W':
      send_limit_switch_rising_edges();
      break;
    default:
      Serial.println("Invalid command!");
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Send Limit Switch Detections
////////////////////////////////////////////////////////////////////////////////
void send_limit_switch_rising_edges()
{
  String msg = "w,";
  array_to_message(limit_switch_rising_edges, NUM_JOINTS, &msg);
  Serial.println(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Send status
/// Encoder / motor steps out of step
/// Limit switch triggered
////////////////////////////////////////////////////////////////////////////////
void send_error_status()
{
  String msg = "q,";

  // Calculate difference in joint positions based on motor steps and encoders
  bool angles_out_of_limits = false;
  float angle_diffs[NUM_JOINTS];
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    angle_diffs[i] = current_joint_positions_from_steps_rad[i] - current_joint_positions_rad[i];
    if (abs(angle_diffs[i]) > 0.005) {
      angles_out_of_limits = true;
    }
  }

  if (angles_out_of_limits) {
    msg += String("1,");
  } else {
    msg += String("0,");
  }

  // Check if the limit switches have been triggered
  bool limit_switch_triggered = false;
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    if (limit_switch_rising_edges[i] > 0) {
      limit_switch_triggered = true;
    }
  }

  if (limit_switch_triggered) {
    msg += String("1");
  } else {
    msg += String("0");
  }

  Serial.println(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Send encoder counts on request
////////////////////////////////////////////////////////////////////////////////
void send_encoder_counts()
{
  String msg = "e,";
  array_to_message(current_encoder_counts, NUM_JOINTS, &msg);
  Serial.println(msg);
}

void send_joint_positions()
{
  String msg = "p,";
  array_to_message(current_joint_positions_rad, NUM_JOINTS, 4, &msg);
  Serial.println(msg);
}

void handle_enable(String letter, char* data, bool* enables, String name)
{
  String tokens[NUM_JOINTS+1];
  if (!parse_line(data, ",", NUM_JOINTS+1, tokens)) {
    Serial << "Invalid toggle command: " << name << "\n";
    return;
  }

  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    enables[i] = (bool)constrain(tokens[i+1].toInt(), 0, 1);
  }
  Serial.println(letter + ",OK");
}

////////////////////////////////////////////////////////////////////////////////
/// Calculate joint positions based on the current encoder counts
////////////////////////////////////////////////////////////////////////////////
void calculate_current_joint_positions()
{
  read_encoder_counts(current_encoder_counts);

  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    current_joint_positions_from_steps_rad[i] = (joint_pos_limits_rad[i]) + enc_dir[i] * (float)(current_motor_steps[i] - joint_pos_limit_motor_steps[i]) / (float)motor_steps_per_rad[i];
    current_joint_positions_rad[i] = (joint_pos_limits_rad[i]) + enc_dir[i] * (float)(current_encoder_counts[i] - joint_pos_limit_encoder_count[i]) / (float)encoder_counts_per_rad[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the desired joint positions
////////////////////////////////////////////////////////////////////////////////
void set_desired_joint_positions(char* data)
{
  // Parse the command string
  String tokens[NUM_JOINTS+1];
  if (!parse_line(data, ",", NUM_JOINTS+1, tokens)) {
    Serial.println("Invalid set joint command.");
    return;
  }

  // Set the desired joint positions, clamp/constrain to valid ranges
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    desired_joint_positions_rad[i] = constrain(tokens[i+1].toFloat(), joint_neg_limits_rad[i], joint_pos_limits_rad[i]);
  }

  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    if (current_state[i] == INACTIVE) {
      current_state[i] = SET_DIRECTION;
    }
  }

  Serial.println("d,OK");
}

////////////////////////////////////////////////////////////////////////////////
/// Write counts to encoders (using during calibration)
////////////////////////////////////////////////////////////////////////////////
void calibrate_encoders(char* data)
{
  String tokens[NUM_JOINTS+1];
  if (!parse_line(data, ",", NUM_JOINTS+1, tokens)) {
    Serial.println("Invalid calibrate encoder command.");
    return;
  }

  int counts[NUM_JOINTS];
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    counts[i] = tokens[i+1].toInt();
  }
  write_encoder_counts(counts);

  // Set the motor counts
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    current_motor_steps[i] = counts[i];// / encoder_mults[i];
  }
  Serial.println("c,OK");
}

////////////////////////////////////////////////////////////////////////////////
/// State machine to achieve desired orientation
////////////////////////////////////////////////////////////////////////////////
void control_loop_motor_steps()
{
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    int bit_delay = 800;

    // Compute the error signal
    joint_positions_err_rad[i] = desired_joint_positions_rad[i] - current_joint_positions_from_steps_rad[i];

    // Compute the absolute number of motor steps (not considering direction)
    desired_motor_steps[i] = round(abs(joint_positions_err_rad[i]) * (float)motor_steps_per_rad[i]);

    // Determine motor direction based on sign of joint position error and
    // direction of increasing encoder count.
    if (joint_positions_err_rad[i] < 0) {
      desired_motor_dirs[i] = enc_dir[i] < 0 ? LOW : HIGH;
    } else {
      desired_motor_dirs[i] = enc_dir[i] < 0 ? HIGH : LOW;
    }
    if (rot_dirs[i] == 0) { // Handle different rotation direction
      desired_motor_dirs[i] = !desired_motor_dirs[i];
    }

    // Compute the desired encoder count at the desired position
    desired_encoder_counts[i] = round((joint_pos_limits_rad[i] + enc_dir[i] * desired_joint_positions_rad[i]) * (float)encoder_counts_per_rad[i]);
    encoder_count_err[i] = desired_encoder_counts[i] - current_encoder_counts[i];

    if (desired_motor_dirs[i] != prev_desired_motor_dirs[i]) {
      current_state[i] = SET_DIRECTION;
    }

    // If the control loop is disabled, don't allow the changing of output
    // signals, but allow for the previous calculations in this loop.
    if (!enable_control_loop[i]) continue;

    switch(current_state[i]) {
      case INACTIVE:
        digitalWrite(step_pins[i], HIGH); // Make sure step pin high on reset
        break;
      case SET_DIRECTION:
        digitalWrite(dir_pins[i], desired_motor_dirs[i]);
        next_state[i] = SET_DIRECTION_DELAY;
        delay_until[i] = time + bit_delay;
        break;
      case SET_DIRECTION_DELAY:
        if (time >= delay_until[i]) {
          next_state[i] = TOGGLE_STEP_LOW;
        }
        break;
      case TOGGLE_STEP_LOW:
        digitalWrite(step_pins[i], LOW);
        --desired_motor_steps[i]; // count motor steps

        if (desired_motor_dirs[i] == 0) {
          if (rot_dirs[i] == 1) {
            current_motor_steps[i]++;
          } else {
            current_motor_steps[i]--;
          }
        } else {
          if (rot_dirs[i] == 1) {
            current_motor_steps[i]--;
          } else {
            current_motor_steps[i]++;
          }
        }

        next_state[i] = TOGGLE_STEP_LOW_DELAY;
        delay_until[i] = time + bit_delay;
        break;
      case TOGGLE_STEP_LOW_DELAY:
        if (time >= delay_until[i]) {
          next_state[i] = TOGGLE_STEP_HIGH;
        }
        break;
      case TOGGLE_STEP_HIGH:
        digitalWrite(step_pins[i], HIGH);
        next_state[i] = TOGGLE_STEP_HIGH_DELAY;
        delay_until[i] = time + bit_delay;
        break;
      case TOGGLE_STEP_HIGH_DELAY:
        if (time >= delay_until[i]) {
          if (desired_motor_steps[i] > 0) {
            next_state[i] = TOGGLE_STEP_LOW;
          } else {
            next_state[i] = INACTIVE;
          }
        }
        break;
      default:
        next_state[i] = INACTIVE;
        break;
    }
    current_state[i] = next_state[i];
    prev_desired_motor_dirs[i] = desired_motor_dirs[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Read individual bytes as they are received
////////////////////////////////////////////////////////////////////////////////
void process_incoming_serial()
{
  // Only read a single byte at a time to keep serial comms from slowing down
  // the rest of the operations.
  if (Serial.available () > 0) {
    process_byte(Serial.read());
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Process received bytes and call process_incoming_message when a complete
/// message has been received.
////////////////////////////////////////////////////////////////////////////////
void process_byte(const byte inByte)
{
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':   // end of text
      input_line [input_pos] = 0;  // terminating null byte

      // Full command received, process it.
      process_incoming_message(input_line);

      // reset buffer for next time
      input_pos = 0;
      break;
    case '\r':   // discard carriage return
      break;
    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1)) {
        input_line [input_pos++] = inByte;
      }
      break;
  }
}

void update_limit_switch_states()
{
  bool disable_control_loops = false;
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    int reading = digitalRead(limit_switches[i]);
    if (reading != last_limit_switch_states[i]) {
      last_debounce_times[i] = time;
    }

    if (time - last_debounce_times[i] > debounce_delay) {
      // if the limit switch state has changed:
      if (reading != limit_switch_states[i]) {
        // if the limit switch has changed from LOW to HIGH, disable all
        // control loops (i.e., Rising edge detection)
        if (reading == HIGH) {
          disable_control_loops = true;
          ++limit_switch_rising_edges[i];
        }

        // Update the state with the latest reading
        limit_switch_states[i] = reading;
      }
    }
    last_limit_switch_states[i] = reading;
  }

  if (disable_control_loops) {
    for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
      enable_control_loop[i] = false;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Print debug values in a timed loop
////////////////////////////////////////////////////////////////////////////////
void print_debug_loop(unsigned long period, unsigned int i)
{
  // Print in loop example
  if (time - prev_print_time[i] > period) {
    Serial << "------- " << time << "\n";
    Serial << "Joint: " << i+1 << "\n";
    Serial << "Control Loop Enabled: " << enable_control_loop[i] << "\n";
    Serial << "State: " << current_state[i] << "\n";
    Serial.print("Current rad (encoder): "); Serial.println(current_joint_positions_rad[i], 5);
    Serial.print("Current rad (motor): "); Serial.println(current_joint_positions_from_steps_rad[i], 5);
    Serial << "Desired rad: " << desired_joint_positions_rad[i] << "\n";
    Serial << "Limit Switch: " << limit_switch_states[i] << "\n";
    prev_print_time[i] = time;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Manually step the motor (used during testing)
////////////////////////////////////////////////////////////////////////////////
void manual_steps(char* data)
{
  String tokens[4];
  if (!parse_line(data, ",", 4, tokens)) {
    Serial.println("Invalid manual step command.");
    return;
  }

  int joint_id = tokens[1].toInt();
  int direction = tokens[2].toInt();
  int steps = tokens[3].toInt();

  int dir_pin = dir_pins[joint_id-1];
  int step_pin = step_pins[joint_id-1];

  // Update encoder counts
  read_encoder_counts(current_encoder_counts);
  Serial << "Joint " << joint_id << ", Starting Encoder Count: " << current_encoder_counts[joint_id-1] << "\n";
  int expected_encoder_count = 0;
  int encoder_counts = round((float)steps * encoder_mults[joint_id-1]);

  if (direction == 0) {
    digitalWrite(dir_pin, LOW);
    expected_encoder_count = current_encoder_counts[joint_id-1] + rot_dir_sign[joint_id-1] * encoder_counts;
  } else {
    digitalWrite(dir_pin, HIGH);
    expected_encoder_count = current_encoder_counts[joint_id-1] - rot_dir_sign[joint_id-1] * encoder_counts;
  }
  delayMicroseconds(1000);

  for (int i = 0; i < steps; i++) {
    digitalWrite(step_pin, LOW);
    delayMicroseconds(1000);
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1000);
  }

  delayMicroseconds(20000);
  read_encoder_counts(current_encoder_counts);
  Serial << "Ending Encoder Count: " << current_encoder_counts[joint_id-1] << "\n";
  Serial << "Encoder Count Error: " << current_encoder_counts[joint_id-1] - expected_encoder_count << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Legacy function for driving to limit switches
/// TODO: Replace with newer version.
////////////////////////////////////////////////////////////////////////////////
void drive_to_limit_switches(char* data)
{
  // Disable the normal control loops
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    enable_control_loop[i] = false;
  }

  // Reset limit switch rising edges
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    limit_switch_states[i] = 1;
    last_limit_switch_states[i] = 1;
    limit_switch_rising_edges[i] = 0;
  }

  // Set desired angles to limits
  for (unsigned int i = 0; i < NUM_JOINTS; ++i) {
    if (cal_dirs[i] == 0) {
      desired_joint_positions_rad[i] = joint_neg_limits_rad[i];
    } else {
      desired_joint_positions_rad[i] = joint_pos_limits_rad[i];
    }
  }

  String tokens[NUM_JOINTS+2];
  if (!parse_line(data, ",", NUM_JOINTS+2, tokens)) {
    Serial.println("Invalid calibrate encoder command.");
    return;
  }

  int J1step = tokens[1].toInt();
  int J2step = tokens[2].toInt();
  int J3step = tokens[3].toInt();
  int J4step = tokens[4].toInt();
  int J5step = tokens[5].toInt();
  int J6step = tokens[6].toInt();

  float SpeedIn = tokens[7].toFloat();

  int J1caldir = 0;
  int J2caldir = 0;
  int J3caldir = 1;
  int J4caldir = 0;
  int J5caldir = 0;
  int J6caldir = 1;

  //RESET COUNTERS
  int J1done = 0;
  int J2done = 0;
  int J3done = 0;
  int J4done = 0;
  int J5done = 0;
  int J6done = 0;

  String J1calStat = "0";

  //SET DIRECTIONS
  // J1 //
  if (J1rotdir == 1 && J1caldir == 1) {
    digitalWrite(J1dirPin, LOW);
  }
  else if (J1rotdir == 0 && J1caldir == 1) {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1rotdir == 1 && J1caldir == 0) {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1rotdir == 0 && J1caldir == 0) {
    digitalWrite(J1dirPin, LOW);
  }

  // J2 //
  if (J2rotdir == 1 && J2caldir == 1) {
    digitalWrite(J2dirPin, LOW);
  }
  else if (J2rotdir == 0 && J2caldir == 1) {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2rotdir == 1 && J2caldir == 0) {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2rotdir == 0 && J2caldir == 0) {
    digitalWrite(J2dirPin, LOW);
  }

  // J3 //
  if (J3rotdir == 1 && J3caldir == 1) {
    digitalWrite(J3dirPin, LOW);
  }
  else if (J3rotdir == 0 && J3caldir == 1) {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3rotdir == 1 && J3caldir == 0) {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3rotdir == 0 && J3caldir == 0) {
    digitalWrite(J3dirPin, LOW);
  }

  // J4 //
  if (J4rotdir == 1 && J4caldir == 1) {
    digitalWrite(J4dirPin, LOW);
  }
  else if (J4rotdir == 0 && J4caldir == 1) {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4rotdir == 1 && J4caldir == 0) {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4rotdir == 0 && J4caldir == 0) {
    digitalWrite(J4dirPin, LOW);
  }

  // J5 //
  if (J5rotdir == 1 && J5caldir == 1) {
    digitalWrite(J5dirPin, LOW);
  }
  else if (J5rotdir == 0 && J5caldir == 1) {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5rotdir == 1 && J5caldir == 0) {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5rotdir == 0 && J5caldir == 0) {
    digitalWrite(J5dirPin, LOW);
  }

  // J6 //
  if (J6rotdir == 1 && J6caldir == 1) {
    digitalWrite(J6dirPin, LOW);
  }
  else if (J6rotdir == 0 && J6caldir == 1) {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6rotdir == 1 && J6caldir == 0) {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6rotdir == 0 && J6caldir == 0) {
    digitalWrite(J6dirPin, LOW);
  }

  float AdjSpeed = (SpeedIn / 100);
  float CalcRegSpeed = ((SpeedMult * 2) / AdjSpeed);
  int Speed = int(CalcRegSpeed);

  //DRIVE MOTORS FOR CALIBRATION
  for (int i = 0; i <= 6; i++) {
    while (digitalRead(J1calPin) == LOW && J1done < J1step || digitalRead(J2calPin) == LOW && J2done < J2step || digitalRead(J3calPin) == LOW && J3done < J3step || digitalRead(J4calPin) == LOW && J4done < J4step || digitalRead(J5calPin) == LOW && J5done < J5step || digitalRead(J6calPin) == LOW && J6done < J6step)
    {
      if (J1done < J1step && (digitalRead(J1calPin) == LOW))
      {
        digitalWrite(J1stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J1done < J1step && (digitalRead(J1calPin) == LOW))
      {
        digitalWrite(J1stepPin, HIGH);
        J1done = ++J1done;
      }
      delayMicroseconds(5);
      if (J2done < J2step && (digitalRead(J2calPin) == LOW))
      {
        digitalWrite(J2stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J2done < J2step && (digitalRead(J2calPin) == LOW))
      {
        digitalWrite(J2stepPin, HIGH);
        J2done = ++J2done;
      }
      delayMicroseconds(5);
      if (J3done < J3step && (digitalRead(J3calPin) == LOW))
      {
        digitalWrite(J3stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J3done < J3step && (digitalRead(J3calPin) == LOW))
      {
        digitalWrite(J3stepPin, HIGH);
        J3done = ++J3done;
      }
      delayMicroseconds(5);
      if (J4done < J4step && (digitalRead(J4calPin) == LOW))
      {
        digitalWrite(J4stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J4done < J4step && (digitalRead(J4calPin) == LOW))
      {
        digitalWrite(J4stepPin, HIGH);
        J4done = ++J4done;
      }
      delayMicroseconds(5);
      if (J5done < J5step && (digitalRead(J5calPin) == LOW))
      {
        digitalWrite(J5stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J5done < J5step && (digitalRead(J5calPin) == LOW))
      {
        digitalWrite(J5stepPin, HIGH);
        J5done = ++J5done;;
      }
      delayMicroseconds(5);
      if (J6done < J6step && (digitalRead(J6calPin) == LOW))
      {
        digitalWrite(J6stepPin, LOW);
      }
      delayMicroseconds(5);
      if (J6done < J6step && (digitalRead(J6calPin) == LOW))
      {
        digitalWrite(J6stepPin, HIGH);
        J6done = ++J6done;
      }
      ///////////////DELAY BEFORE RESTARTING LOOP
      delayMicroseconds(Speed);
    }
    delayMicroseconds(10);
  }
  //OVERDRIVE
  int OvrDrv = 0;
  while (OvrDrv <= 25)
  {
    if (J1step > 0)
    {
      digitalWrite(J1stepPin, LOW);
    }
    if (J2step > 0)
    {
      digitalWrite(J2stepPin, LOW);
    }
    if (J3step > 0)
    {
      digitalWrite(J3stepPin, LOW);
    }
    if (J4step > 0)
    {
      digitalWrite(J4stepPin, LOW);
    }
    if (J5step > 0)
    {
      digitalWrite(J5stepPin, LOW);
    }
    if (J6step > 0)
    {
      digitalWrite(J6stepPin, LOW);
    }
    ///////////////DELAY AND SET HIGH
    delayMicroseconds(Speed);
    if (J1step > 0)
    {
      digitalWrite(J1stepPin, HIGH);
    }
    if (J2step > 0)
    {
      digitalWrite(J2stepPin, HIGH);
    }
    if (J3step > 0)
    {
      digitalWrite(J3stepPin, HIGH);
    }
    if (J4step > 0)
    {
      digitalWrite(J4stepPin, HIGH);
    }
    if (J5step > 0)
    {
      digitalWrite(J5stepPin, HIGH);
    }
    if (J6step > 0)
    {
      digitalWrite(J6stepPin, HIGH);
    }
    OvrDrv = ++OvrDrv;
    ///////////////DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
    delayMicroseconds(Speed);
  }
  //SEE IF ANY SWITCHES NOT MADE
  delay(500);
  ///
  int J1pass = 1;
  int J2pass = 1;
  int J3pass = 1;
  int J4pass = 1;
  int J5pass = 1;
  int J6pass = 1;
  ///
  if (J1step > 0) {
    if (digitalRead(J1calPin) == LOW) {
      J1pass = 0;
    }
  }
  if (J2step > 0) {
    if (digitalRead(J2calPin) == LOW) {
      J2pass = 0;
    }
  }
  if (J3step > 0) {
    if (digitalRead(J3calPin) == LOW) {
      J3pass = 0;
    }
  }
  if (J4step > 0) {
    if (digitalRead(J4calPin) == LOW) {
      J4pass = 0;
    }
  }
  if (J5step > 0)
  { if (digitalRead(J5calPin) == LOW) {
      J5pass = 0;
    }
  }
  if (J6step > 0)
  { if (digitalRead(J6calPin) == LOW) {
      J6pass = 0;
    }
  }
  if ((J1pass + J2pass + J3pass + J4pass + J5pass + J6pass) == 6)
  {
    Serial.println("P");
  }
  else
  {
    Serial.println("F");
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Legacy function for stepping the motor
/// TODO: Replace with newer version.
////////////////////////////////////////////////////////////////////////////////
void old_move(char* data)
{
  String tokens[NUM_JOINTS+2];
  if (!parse_line(data, ",", NUM_JOINTS+2, tokens)) {
    Serial.println("Invalid calibrate encoder command.");
    return;
  }

  int J1curStep = J1encPos.read() / J1encMult;
  int J2curStep = J2encPos.read() / J2encMult;
  int J3curStep = J3encPos.read() / J3encMult;
  int J4curStep = J4encPos.read() / J4encMult;
  int J5curStep = J5encPos.read() / J5encMult;
  int J6curStep = J6encPos.read() / J6encMult;

  int J1dir = 1;
  int J2dir = 1;
  int J3dir = 0;
  int J4dir = 1;
  int J5dir = 1;
  int J6dir = 0;
  int TRdir = 0;

  int J1step = tokens[1].toInt();
  int J2step = tokens[2].toInt();
  int J3step = tokens[3].toInt();
  int J4step = tokens[4].toInt();
  int J5step = tokens[5].toInt();
  int J6step = tokens[6].toInt();
  int TRstep = 0;

  float SpeedIn = tokens[7].toFloat();
  float ACCdur = 15;
  float ACCspd = 10;
  float DCCdur = 20;
  float DCCspd = 5;
  int J1tarStep = 0; //inData.substring(J1Tstart + 1, J2Tstart).toInt();
  int J2tarStep = 0; //inData.substring(J2Tstart + 1, J3Tstart).toInt();
  int J3tarStep = 0; //inData.substring(J3Tstart + 1, J4Tstart).toInt();
  int J4tarStep = 0; //inData.substring(J4Tstart + 1, J5Tstart).toInt();
  int J5tarStep = 0; //inData.substring(J5Tstart + 1, J6Tstart).toInt();
  int J6tarStep = 0; //inData.substring(J6Tstart + 1).toInt();



  //FIND HIGHEST STEP
  int HighStep = J1step;
  if (J2step > HighStep)
  {
    HighStep = J2step;
  }
  if (J3step > HighStep)
  {
    HighStep = J3step;
  }
  if (J4step > HighStep)
  {
    HighStep = J4step;
  }
  if (J5step > HighStep)
  {
    HighStep = J5step;
  }
  if (J6step > HighStep)
  {
    HighStep = J6step;
  }
  if (TRstep > HighStep)
  {
    HighStep = TRstep;
  }

  //FIND ACTIVE JOINTS
  int J1active = 0;
  int J2active = 0;
  int J3active = 0;
  int J4active = 0;
  int J5active = 0;
  int J6active = 0;
  int TRactive = 0;
  int Jactive = 0;

  if (J1step >= 1)
  {
    J1active = 1;
  }
  if (J2step >= 1)
  {
    J2active = 1;
  }
  if (J3step >= 1)
  {
    J3active = 1;
  }
  if (J4step >= 1)
  {
    J4active = 1;
  }
  if (J5step >= 1)
  {
    J5active = 1;
  }
  if (J6step >= 1)
  {
    J6active = 1;
  }
  if (TRstep >= 1)
  {
    TRactive = 1;
  }
  Jactive = (J1active + J2active + J3active + J4active + J5active + J6active + TRactive);

  int J1_PE = 0;
  int J2_PE = 0;
  int J3_PE = 0;
  int J4_PE = 0;
  int J5_PE = 0;
  int J6_PE = 0;
  int TR_PE = 0;

  int J1_SE_1 = 0;
  int J2_SE_1 = 0;
  int J3_SE_1 = 0;
  int J4_SE_1 = 0;
  int J5_SE_1 = 0;
  int J6_SE_1 = 0;
  int TR_SE_1 = 0;

  int J1_SE_2 = 0;
  int J2_SE_2 = 0;
  int J3_SE_2 = 0;
  int J4_SE_2 = 0;
  int J5_SE_2 = 0;
  int J6_SE_2 = 0;
  int TR_SE_2 = 0;

  int J1_LO_1 = 0;
  int J2_LO_1 = 0;
  int J3_LO_1 = 0;
  int J4_LO_1 = 0;
  int J5_LO_1 = 0;
  int J6_LO_1 = 0;
  int TR_LO_1 = 0;

  int J1_LO_2 = 0;
  int J2_LO_2 = 0;
  int J3_LO_2 = 0;
  int J4_LO_2 = 0;
  int J5_LO_2 = 0;
  int J6_LO_2 = 0;
  int TR_LO_2 = 0;

  //reset
  int J1cur = 0;
  int J2cur = 0;
  int J3cur = 0;
  int J4cur = 0;
  int J5cur = 0;
  int J6cur = 0;
  int TRcur = 0;

  int J1_PEcur = 0;
  int J2_PEcur = 0;
  int J3_PEcur = 0;
  int J4_PEcur = 0;
  int J5_PEcur = 0;
  int J6_PEcur = 0;
  int TR_PEcur = 0;

  int J1_SE_1cur = 0;
  int J2_SE_1cur = 0;
  int J3_SE_1cur = 0;
  int J4_SE_1cur = 0;
  int J5_SE_1cur = 0;
  int J6_SE_1cur = 0;
  int TR_SE_1cur = 0;

  int J1_SE_2cur = 0;
  int J2_SE_2cur = 0;
  int J3_SE_2cur = 0;
  int J4_SE_2cur = 0;
  int J5_SE_2cur = 0;
  int J6_SE_2cur = 0;
  int TR_SE_2cur = 0;

  int highStepCur = 0;
  float curDelay = 0;


  //SET DIRECTIONS

  /////// J1 /////////
  if (J1dir == 1 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, LOW);
  }
  else if (J1dir == 1 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 1)
  {
    digitalWrite(J1dirPin, HIGH);
  }
  else if (J1dir == 0 && J1rotdir == 0)
  {
    digitalWrite(J1dirPin, LOW);
  }

  /////// J2 /////////
  if (J2dir == 1 && J2rotdir == 1)
  {
    digitalWrite(J2dirPin, LOW);
  }
  else if (J2dir == 1 && J2rotdir == 0)
  {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2dir == 0 && J2rotdir == 1)
  {
    digitalWrite(J2dirPin, HIGH);
  }
  else if (J2dir == 0 && J2rotdir == 0)
  {
    digitalWrite(J2dirPin, LOW);
  }

  /////// J3 /////////
  if (J3dir == 1 && J3rotdir == 1)
  {
    digitalWrite(J3dirPin, LOW);
  }
  else if (J3dir == 1 && J3rotdir == 0)
  {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3dir == 0 && J3rotdir == 1)
  {
    digitalWrite(J3dirPin, HIGH);
  }
  else if (J3dir == 0 && J3rotdir == 0)
  {
    digitalWrite(J3dirPin, LOW);
  }

  /////// J4 /////////
  if (J4dir == 1 && J4rotdir == 1)
  {
    digitalWrite(J4dirPin, LOW);
  }
  else if (J4dir == 1 && J4rotdir == 0)
  {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4dir == 0 && J4rotdir == 1)
  {
    digitalWrite(J4dirPin, HIGH);
  }
  else if (J4dir == 0 && J4rotdir == 0)
  {
    digitalWrite(J4dirPin, LOW);
  }

  /////// J5 /////////
  if (J5dir == 1 && J5rotdir == 1)
  {
    digitalWrite(J5dirPin, LOW);
  }
  else if (J5dir == 1 && J5rotdir == 0)
  {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5dir == 0 && J5rotdir == 1)
  {
    digitalWrite(J5dirPin, HIGH);
  }
  else if (J5dir == 0 && J5rotdir == 0)
  {
    digitalWrite(J5dirPin, LOW);
  }

  /////// J6 /////////
  if (J6dir == 1 && J6rotdir == 1)
  {
    digitalWrite(J6dirPin, LOW);
  }
  else if (J6dir == 1 && J6rotdir == 0)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6dir == 0 && J6rotdir == 1)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  else if (J6dir == 0 && J6rotdir == 0)
  {
    digitalWrite(J6dirPin, LOW);
  }

  /////// TRACK /////////
  if (TRdir == 1 && TRACKrotdir == 1)
  {
    digitalWrite(TRdirPin, LOW);
  }
  else if (TRdir == 1 && TRACKrotdir == 0)
  {
    digitalWrite(TRdirPin, HIGH);
  }
  else if (TRdir == 0 && TRACKrotdir == 1)
  {
    digitalWrite(TRdirPin, HIGH);
  }
  else if (TRdir == 0 && TRACKrotdir == 0)
  {
    digitalWrite(TRdirPin, LOW);
  }



  /////CALC SPEEDS//////
  float ACCStep = (HighStep * (ACCdur / 100));
  float DCCStep = HighStep - (HighStep * (DCCdur / 100));
  float AdjSpeed = (SpeedIn / 100);
  //REG SPEED
  float CalcRegSpeed = (SpeedMult / AdjSpeed);
  int REGSpeed = int(CalcRegSpeed);

  //ACC SPEED
  float ACCspdT = (ACCspd / 100);
  float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
  float ACCSpeed = (CalcACCSpeed);
  float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

  //DCC SPEED
  float DCCspdT = (DCCspd / 100);
  float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
  float DCCSpeed = (CalcDCCSpeed);
  float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
  DCCSpeed = REGSpeed;




  ///// DRIVE MOTORS /////
  while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step || TRcur < TRstep)
    //while (J1curStep < J1tarStep || J1curStep != J1tarStep)
  {

    ////DELAY CALC/////
    if (highStepCur <= ACCStep)
    {
      curDelay = (ACCSpeed / Jactive);
      ACCSpeed = ACCSpeed + ACCinc;
    }
    else if (highStepCur >= DCCStep)
    {
      curDelay = (DCCSpeed / Jactive);
      DCCSpeed = DCCSpeed + DCCinc;
    }
    else
    {
      curDelay = (REGSpeed / Jactive);
    }

    /////// J1 ////////////////////////////////
    ///find pulse every
    if (J1cur < J1step)
    {
      J1_PE = (HighStep / J1step);
      ///find left over 1
      J1_LO_1 = (HighStep - (J1step * J1_PE));
      ///find skip 1
      if (J1_LO_1 > 0)
      {
        J1_SE_1 = (HighStep / J1_LO_1);
      }
      else
      {
        J1_SE_1 = 0;
      }
      ///find left over 2
      if (J1_SE_1 > 0)
      {
        J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
      }
      else
      {
        J1_LO_2 = 0;
      }
      ///find skip 2
      if (J1_LO_2 > 0)
      {
        J1_SE_2 = (HighStep / J1_LO_2);
      }
      else
      {
        J1_SE_2 = 0;
      }
      /////////  J1  ///////////////
      if (J1_SE_2 == 0)
      {
        J1_SE_2cur = (J1_SE_2 + 1);
      }
      if (J1_SE_2cur != J1_SE_2)
      {
        J1_SE_2cur = ++J1_SE_2cur;
        if (J1_SE_1 == 0)
        {
          J1_SE_1cur = (J1_SE_1 + 1);
        }
        if (J1_SE_1cur != J1_SE_1)
        {
          J1_SE_1cur = ++J1_SE_1cur;
          J1_PEcur = ++J1_PEcur;
          if (J1_PEcur == J1_PE)
          {
            J1cur = ++J1cur;
            J1_PEcur = 0;
            digitalWrite(J1stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J1stepPin, HIGH);
          }
        }
        else
        {
          J1_SE_1cur = 0;
        }
      }
      else
      {
        J1_SE_2cur = 0;
      }
    }

    /////// J2 ////////////////////////////////
    ///find pulse every
    if (J2cur < J2step)
    {
      J2_PE = (HighStep / J2step);
      ///find left over 1
      J2_LO_1 = (HighStep - (J2step * J2_PE));
      ///find skip 1
      if (J2_LO_1 > 0)
      {
        J2_SE_1 = (HighStep / J2_LO_1);
      }
      else
      {
        J2_SE_1 = 0;
      }
      ///find left over 2
      if (J2_SE_1 > 0)
      {
        J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
      }
      else
      {
        J2_LO_2 = 0;
      }
      ///find skip 2
      if (J2_LO_2 > 0)
      {
        J2_SE_2 = (HighStep / J2_LO_2);
      }
      else
      {
        J2_SE_2 = 0;
      }
      /////////  J2  ///////////////
      if (J2_SE_2 == 0)
      {
        J2_SE_2cur = (J2_SE_2 + 1);
      }
      if (J2_SE_2cur != J2_SE_2)
      {
        J2_SE_2cur = ++J2_SE_2cur;
        if (J2_SE_1 == 0)
        {
          J2_SE_1cur = (J2_SE_1 + 1);
        }
        if (J2_SE_1cur != J2_SE_1)
        {
          J2_SE_1cur = ++J2_SE_1cur;
          J2_PEcur = ++J2_PEcur;
          if (J2_PEcur == J2_PE)
          {
            J2cur = ++J2cur;
            J2_PEcur = 0;
            digitalWrite(J2stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J2stepPin, HIGH);
          }
        }
        else
        {
          J2_SE_1cur = 0;
        }
      }
      else
      {
        J2_SE_2cur = 0;
      }
    }

    /////// J3 ////////////////////////////////
    ///find pulse every
    if (J3cur < J3step)
    {
      J3_PE = (HighStep / J3step);
      ///find left over 1
      J3_LO_1 = (HighStep - (J3step * J3_PE));
      ///find skip 1
      if (J3_LO_1 > 0)
      {
        J3_SE_1 = (HighStep / J3_LO_1);
      }
      else
      {
        J3_SE_1 = 0;
      }
      ///find left over 2
      if (J3_SE_1 > 0)
      {
        J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
      }
      else
      {
        J3_LO_2 = 0;
      }
      ///find skip 2
      if (J3_LO_2 > 0)
      {
        J3_SE_2 = (HighStep / J3_LO_2);
      }
      else
      {
        J3_SE_2 = 0;
      }
      /////////  J3  ///////////////
      if (J3_SE_2 == 0)
      {
        J3_SE_2cur = (J3_SE_2 + 1);
      }
      if (J3_SE_2cur != J3_SE_2)
      {
        J3_SE_2cur = ++J3_SE_2cur;
        if (J3_SE_1 == 0)
        {
          J3_SE_1cur = (J3_SE_1 + 1);
        }
        if (J3_SE_1cur != J3_SE_1)
        {
          J3_SE_1cur = ++J3_SE_1cur;
          J3_PEcur = ++J3_PEcur;
          if (J3_PEcur == J3_PE)
          {
            J3cur = ++J3cur;
            J3_PEcur = 0;
            digitalWrite(J3stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J3stepPin, HIGH);
          }
        }
        else
        {
          J3_SE_1cur = 0;
        }
      }
      else
      {
        J3_SE_2cur = 0;
      }
    }

    /////// J4 ////////////////////////////////
    ///find pulse every
    if (J4cur < J4step)
    {
      J4_PE = (HighStep / J4step);
      ///find left over 1
      J4_LO_1 = (HighStep - (J4step * J4_PE));
      ///find skip 1
      if (J4_LO_1 > 0)
      {
        J4_SE_1 = (HighStep / J4_LO_1);
      }
      else
      {
        J4_SE_1 = 0;
      }
      ///find left over 2
      if (J4_SE_1 > 0)
      {
        J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
      }
      else
      {
        J4_LO_2 = 0;
      }
      ///find skip 2
      if (J4_LO_2 > 0)
      {
        J4_SE_2 = (HighStep / J4_LO_2);
      }
      else
      {
        J4_SE_2 = 0;
      }
      /////////  J4  ///////////////
      if (J4_SE_2 == 0)
      {
        J4_SE_2cur = (J4_SE_2 + 1);
      }
      if (J4_SE_2cur != J4_SE_2)
      {
        J4_SE_2cur = ++J4_SE_2cur;
        if (J4_SE_1 == 0)
        {
          J4_SE_1cur = (J4_SE_1 + 1);
        }
        if (J4_SE_1cur != J4_SE_1)
        {
          J4_SE_1cur = ++J4_SE_1cur;
          J4_PEcur = ++J4_PEcur;
          if (J4_PEcur == J4_PE)
          {
            J4cur = ++J4cur;
            J4_PEcur = 0;
            digitalWrite(J4stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J4stepPin, HIGH);
          }
        }
        else
        {
          J4_SE_1cur = 0;
        }
      }
      else
      {
        J4_SE_2cur = 0;
      }
    }

    /////// J5 ////////////////////////////////
    ///find pulse every
    if (J5cur < J5step)
    {
      J5_PE = (HighStep / J5step);
      ///find left over 1
      J5_LO_1 = (HighStep - (J5step * J5_PE));
      ///find skip 1
      if (J5_LO_1 > 0)
      {
        J5_SE_1 = (HighStep / J5_LO_1);
      }
      else
      {
        J5_SE_1 = 0;
      }
      ///find left over 2
      if (J5_SE_1 > 0)
      {
        J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
      }
      else
      {
        J5_LO_2 = 0;
      }
      ///find skip 2
      if (J5_LO_2 > 0)
      {
        J5_SE_2 = (HighStep / J5_LO_2);
      }
      else
      {
        J5_SE_2 = 0;
      }
      /////////  J5  ///////////////
      if (J5_SE_2 == 0)
      {
        J5_SE_2cur = (J5_SE_2 + 1);
      }
      if (J5_SE_2cur != J5_SE_2)
      {
        J5_SE_2cur = ++J5_SE_2cur;
        if (J5_SE_1 == 0)
        {
          J5_SE_1cur = (J5_SE_1 + 1);
        }
        if (J5_SE_1cur != J5_SE_1)
        {
          J5_SE_1cur = ++J5_SE_1cur;
          J5_PEcur = ++J5_PEcur;
          if (J5_PEcur == J5_PE)
          {
            J5cur = ++J5cur;
            J5_PEcur = 0;
            digitalWrite(J5stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J5stepPin, HIGH);
          }
        }
        else
        {
          J5_SE_1cur = 0;
        }
      }
      else
      {
        J5_SE_2cur = 0;
      }
    }

    /////// J6 ////////////////////////////////
    ///find pulse every
    if (J6cur < J6step)
    {
      J6_PE = (HighStep / J6step);
      ///find left over 1
      J6_LO_1 = (HighStep - (J6step * J6_PE));
      ///find skip 1
      if (J6_LO_1 > 0)
      {
        J6_SE_1 = (HighStep / J6_LO_1);
      }
      else
      {
        J6_SE_1 = 0;
      }
      ///find left over 2
      if (J6_SE_1 > 0)
      {
        J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
      }
      else
      {
        J6_LO_2 = 0;
      }
      ///find skip 2
      if (J6_LO_2 > 0)
      {
        J6_SE_2 = (HighStep / J6_LO_2);
      }
      else
      {
        J6_SE_2 = 0;
      }
      /////////  J6  ///////////////
      if (J6_SE_2 == 0)
      {
        J6_SE_2cur = (J6_SE_2 + 1);
      }
      if (J6_SE_2cur != J6_SE_2)
      {
        J6_SE_2cur = ++J6_SE_2cur;
        if (J6_SE_1 == 0)
        {
          J6_SE_1cur = (J6_SE_1 + 1);
        }
        if (J6_SE_1cur != J6_SE_1)
        {
          J6_SE_1cur = ++J6_SE_1cur;
          J6_PEcur = ++J6_PEcur;
          if (J6_PEcur == J6_PE)
          {
            J6cur = ++J6cur;
            J6_PEcur = 0;
            digitalWrite(J6stepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(J6stepPin, HIGH);
          }
        }
        else
        {
          J6_SE_1cur = 0;
        }
      }
      else
      {
        J6_SE_2cur = 0;
      }
    }

    /////// TR ////////////////////////////////
    ///find pulse every
    if (TRcur < TRstep)
    {
      TR_PE = (HighStep / TRstep);
      ///find left over 1
      TR_LO_1 = (HighStep - (TRstep * TR_PE));
      ///find skip 1
      if (TR_LO_1 > 0)
      {
        TR_SE_1 = (HighStep / TR_LO_1);
      }
      else
      {
        TR_SE_1 = 0;
      }
      ///find left over 2
      if (TR_SE_1 > 0)
      {
        TR_LO_2 = HighStep - ((TRstep * TR_PE) + ((TRstep * TR_PE) / TR_SE_1));
      }
      else
      {
        TR_LO_2 = 0;
      }
      ///find skip 2
      if (TR_LO_2 > 0)
      {
        TR_SE_2 = (HighStep / TR_LO_2);
      }
      else
      {
        TR_SE_2 = 0;
      }
      /////////  TR  ///////////////
      if (TR_SE_2 == 0)
      {
        TR_SE_2cur = (TR_SE_2 + 1);
      }
      if (TR_SE_2cur != TR_SE_2)
      {
        TR_SE_2cur = ++TR_SE_2cur;
        if (TR_SE_1 == 0)
        {
          TR_SE_1cur = (TR_SE_1 + 1);
        }
        if (TR_SE_1cur != TR_SE_1)
        {
          TR_SE_1cur = ++TR_SE_1cur;
          TR_PEcur = ++TR_PEcur;
          if (TR_PEcur == TR_PE)
          {
            TRcur = ++TRcur;
            TR_PEcur = 0;
            digitalWrite(TRstepPin, LOW);
            delayMicroseconds(curDelay);
            digitalWrite(TRstepPin, HIGH);
          }
        }
        else
        {
          TR_SE_1cur = 0;
        }
      }
      else
      {
        TR_SE_2cur = 0;
      }
    }


    // inc cur step
    highStepCur = ++highStepCur;
    delayMicroseconds(200);


  }

  ////////// check for stalled motor
  int ErrorTrue = 0;
  String ErrorCode = "00";
  String J1error = "0";
  String J2error = "0";
  String J3error = "0";
  String J4error = "0";
  String J5error = "0";
  String J6error = "0";

  J1curStep = J1encPos.read() / J1encMult;
  J2curStep = J2encPos.read() / J2encMult;
  J3curStep = J3encPos.read() / J3encMult;
  J4curStep = J4encPos.read() / J4encMult;
  J5curStep = J5encPos.read() / J5encMult;
  J6curStep = J6encPos.read() / J6encMult;

  if (abs(J1curStep - J1tarStep) >  J1encMult / EncDiv)
  {
    J1error = "1";
    ErrorTrue = 1;
  }
  if (abs(J2curStep - J2tarStep) >  J2encMult / EncDiv)
  {
    J2error = "1";
    ErrorTrue = 1;
  }
  if (abs(J3curStep - J3tarStep) >  J3encMult / EncDiv)
  {
    J3error = "1";
    ErrorTrue = 1;
  }
  if (abs(J4curStep - J4tarStep) >  J4encMult / EncDiv)
  {
    J4error = "1";
    ErrorTrue = 1;
  }
  if (abs(J5curStep - J5tarStep) >  J5encMult / EncDiv)
  {
    J5error = "1";
    ErrorTrue = 1;
  }
  if (abs(J6curStep - J6tarStep) >  J6encMult / EncDiv)
  {
    J6error = "1";
    ErrorTrue = 1;
  }

  if (ErrorTrue == 1)
  {
    ErrorCode = "01" + J1error + J2error + J3error + J4error + J5error + J6error + "A" + String(J1curStep) + "B" + String(J2curStep) + "C" + String(J3curStep) + "D" + String(J4curStep) + "E" + String(J5curStep) + "F" + String(J6curStep);
  }
  else if (ErrorTrue == 0)
  {
    J1encPos.write(J1tarStep * J1encMult);
    J2encPos.write(J2tarStep * J2encMult);
    J3encPos.write(J3tarStep * J3encMult);
    J4encPos.write(J4tarStep * J4encMult);
    J5encPos.write(J5tarStep * J5encMult);
    J6encPos.write(J6tarStep * J6encMult);
  }


  Serial.print(ErrorCode);
  Serial.println();
  ////////MOVE COMPLETE///////////
}
