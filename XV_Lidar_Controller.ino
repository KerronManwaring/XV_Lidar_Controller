/* 

  Copyright 2014-2016 James LeRoy getSurreal
  SurrealXV Lidar Controller 
  
  Modified by Kerron Manwaring to run on Arduino Due 
*/
/*


*/
#include <PID.h>
#include <SerialCommand.h>

const int N_ANGLES = 360;                // # of angles (0..359)
const int SHOW_ALL_ANGLES = N_ANGLES;    // value means 'display all angle data, 0..359'


const int motor_pwm_pin = 13;            // pin connected to mosfet for motor speed control
const int ledPin = 12;


boolean ledState = LOW;
double rpm_setpoint = 250;          // desired RPM (uses double to be compatible with PID library)
double rpm_min = 200;
double rpm_max = 300;
double pwm_max = 100;              // max analog value.  probably never needs to change from 1023
double pwm_min = 1023;              // min analog pulse value to spin the motor
int sample_time = 20;             // how often to calculate the PID values

// PID tuning values
double Kp = 2.0;
double Ki = 1.0;
double Kd = 0.0;

boolean motor_enable = true;         // to spin the laser or not.  No data when not spinning
boolean raw_data = false;            // to retransmit the seiral data to the USB port
boolean show_dist = true;            // controlled by ShowDist and HideDist commands
boolean show_points = false;         // to show or hide data as cartesian x,y points 
boolean show_rpm = false;            // controlled by ShowRPM and HideRPM commands
boolean show_interval = false;       // true = show time interval, once per revolution, at angle=0
boolean show_errors = false;         // Show CRC, signal strength and invalid data errors
boolean aryAngles[N_ANGLES];         // array of angles to display


double pwm_val = 500;          // start with ~50% power
double pwm_last;
double motor_rpm;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillis = millis();

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int OFFSET_DATA_DISTANCE_LSB = 0;
const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

int Packet[PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                          // index into 'Packet' array
const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"


const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"

const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = eState_Find_COMMAND;
PID rpmPID(&motor_rpm, &pwm_val, &rpm_setpoint, Kp, Ki, Kd, DIRECT);

uint8_t inByte = 0;  // incoming serial byte
uint8_t motor_rph_high_byte = 0;
uint8_t motor_rph_low_byte = 0;
uint16_t aryDist[N_DATA_QUADS] = {0, 0, 0, 0};   // thre are (4) distances, one for each data quad
// so the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[N_DATA_QUADS] = {0, 0, 0, 0}; // same with 'quality'
uint16_t motor_rph = 0;
uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

SerialCommand sCmd;


// initialization (before 'loop')
void setup() {

  for (int ix = 0; ix < N_ANGLES; ix++) {
    aryAngles[ix] = true;
  }
  pinMode(motor_pwm_pin, OUTPUT);
  Serial.begin(115200); // USB serial
  Serial1.begin(115200);                   // XV LDS data
 // Timer3.initialize(30);                           // set PWM frequency to 32.768kHz

 

  rpmPID.SetOutputLimits(pwm_min, pwm_max);
  rpmPID.SetSampleTime(sample_time);
  rpmPID.SetTunings(Kp, Ki, Kd);
  rpmPID.SetMode(AUTOMATIC);

  initSerialCommands();
  pinMode(ledPin, OUTPUT);

  eState = eState_Find_COMMAND;
  for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // Initialize
    Packet[ixPacket] = 0;
  ixPacket = 0;
}

void loop() {
  
// Do what ever you want here

}

void serialEvent() { // Terminal serial 
  sCmd.readSerial();
}

void serialEvent1() { // Lidar serial
  Lidar();
}

void Lidar() {
  byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

  sCmd.readSerial();  // check for incoming serial commands
  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    if (raw_data)
      Serial.write(inByte);                 // relay

    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          processSpeed();                             // process the speed
          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }
          if (show_dist) {                           // the 'ShowDistance' command is active
            for (int ix = 0; ix < N_DATA_QUADS; ix++) {
              if (aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
                if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...
                  if (show_errors) {                           // if we're supposed to show data errors...
                    Serial.print(F("A,"));
                    Serial.print(startingAngle + ix);
                    Serial.print(F(","));
                    if (aryInvalidDataFlag[ix] & INVALID_DATA_FLAG)
                      Serial.println(F("I"));
                    if (aryInvalidDataFlag[ix] & STRENGTH_WARNING_FLAG)
                      Serial.println(F("S"));
                  }
                }
                else {  
      
                  // show clean data
                     if(!show_points){ 
                      Serial.print(F("A,"));
                      Serial.print(startingAngle + ix);
                      Serial.print(F(","));
                      Serial.print(int(aryDist[ix]));
                      Serial.print(F(","));
                      Serial.println(aryQuality[ix]);
                     }
                  
                     if(show_points){  // show cartesian                 
                      double x = aryDist[ix] * cos((startingAngle + ix)* 3.14/180); 
                      double y = aryDist[ix] * sin((startingAngle + ix)* 3.14/180);
                      Serial.print(F("P,"));                      
                      Serial.print(x);
                      Serial.print(F(","));
                      Serial.println(y);
                     }
                  
                }
              }  // if (aryAngles[startingAngle + ix])
            }  // for (int ix = 0; ix < N_DATA_QUADS; ix++)
          }  // if (show_dist)
        }  // if (eValidatePacket() == 0
        else if (show_errors) {                                // we have encountered a CRC error
          Serial.println(F("C,CRC"));
        }
        // initialize a bunch of stuff before we switch back to State 1
        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          aryDist[ix] = 0;
          aryQuality[ix] = 0;
          aryInvalidDataFlag[ix] = 0;
        }
        for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
          Packet[ixPacket] = 0;
        ixPacket = 0;
        eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte
      }  // if (ixPacket == PACKET_LENGTH)
    }  // if (eState == eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  if (motor_enable) {
    rpmPID.Compute();
    if (pwm_val != pwm_last) {
      //Timer3.pwm(motor_pwm_pin, pwm_val);  // replacement for analogWrite()
      analogWrite(motor_pwm_pin, pwm_val);
      pwm_last = pwm_val;
    }
    motorCheck();
  }  // if (motor_enable)
}  // loop

uint16_t processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);

    if (show_rpm) {
      Serial.print(F("R,"));
      Serial.print((int)motor_rpm);
      Serial.print(F(","));
      Serial.println((int)pwm_val);
    }

    curMillis = millis();
    if (show_interval) {
      Serial.print(F("T,"));                                // Time Interval in ms since last complete revolution
      Serial.println(curMillis - lastMillis);
    }
    lastMillis = curMillis;

  } // if (angle == 0)
  return angle;
}

void processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}

byte processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}

void processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}


byte eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b))
    return VALID_PACKET;                       // okay
  else
    return INVALID_PACKET;                     // non-zero = bad CRC
}


void initSerialCommands() {
  sCmd.addCommand("SetAngle",      setAngle);
  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);
  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);
  sCmd.addCommand("ShowRaw",  showRaw);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowDist",  showDist);
  sCmd.addCommand("ShowPoints",  showPoints);
  sCmd.addCommand("HidePoints",  hidePoints);
  sCmd.addCommand("HideDist",  hideDist);
  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("ShowErrors", showErrors);
  sCmd.addCommand("HideErrors", hideErrors);
  sCmd.addCommand("ShowInterval", showInterval);
  sCmd.addCommand("HideInterval", hideInterval);
  sCmd.addCommand("ShowAll", showAll);
  sCmd.addCommand("HideAll", hideAll);
}
/*
   showAll - Show Dist, Errors, RPM, and Interval data
*/
void showAll() {
  showDist();
  showErrors();
  showRPM();
  showInterval();
}
/*
   hideAll - Hide Dist, Errors, RPM, and Interval data
*/
void hideAll() {
  hideDist();
  hideErrors();
  hideRPM();
  hideInterval();
}
/*
   showInterval - enable display of Time interval (which happens once per revolution, at angle 0
*/
void showInterval() {
  show_interval = true;
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing time interval (ms per revolution)"));
  }
}
/*
   hideInterval - suppress display of Time interval
*/
void hideInterval() {
  show_interval = false;
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding time interval"));
  }
}
/*
   showErrors
*/
void showErrors() {
  show_errors = true;                                  // enable error display
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing errors"));
  }
}
/*
   hideErrors
*/
void hideErrors() {                                    // disable error display
  show_errors = false;
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding errors"));
  }
}
/*
   showRPM
*/
void showRPM() {
  show_rpm = true;
  if (raw_data == true) {
    hideRaw();
  }
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing RPM data"));
  }
}
/*
   hideRPM
*/
void hideRPM() {
  show_rpm = false;
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding RPM data"));
  }
}

void showDist() {
  hideRaw();
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Code,Angle,Distance(mm),Signal strength"));
  }
  show_dist = true;
}

void hideDist() {
  show_dist = false;
  if (show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding Distance data"));
  }
}

void setAngle() {
  char c, *arg;
  boolean syntax_error = false;
  int doing_from_to, from, to, ix, lToken, n_groups = 0;

  for (ix = 0; ix < N_ANGLES; ix++)                      // initialize
    aryAngles[ix] = false;
  doing_from_to = 0;                                     // state = doing 'from'
  // Make sure that there is at least 1 angle or group of angles present
  do {
    arg = sCmd.next();                                   // get the next token
    if (arg == NULL) {                                   // it's empty -- just exit
      sCmd.readSerial();
      arg = sCmd.next();
      break;
    }
    // see if the token has an embedded "-", meaning from - to
    lToken = strlen(arg);                                // get the length of the current token
    for (ix = 0; ix < lToken; ix++) {
      c = arg[ix];
      if (c == ',') {                                    // optional trailing comma
        doing_from_to = 0;
        break;
      }
      else if (c == '-') {                               // optional '-' means "from - to"
        to = 0;
        doing_from_to = 1;                               // from now on, we're doing 'to'
      }
      else if (c == ' ') {                               // ignore blanks
        Serial.println(F("{ }"));
      }
      else if ((c >= '0') && (c <= '9')) {
        if (doing_from_to == 0) {
          from *= 10;
          from += c - '0';
          to = from;                                      // default to = from
          n_groups++;                                     // count the number of active groups (s/b >= 1)
        }
        else {
          to *= 10;
          to += c - '0';
        }
      }
      else {
        syntax_error = true;
        n_groups = 0;
        break;
      }
    }  // for (ix = 0; ix < lToken; ix++)
    // validate 'from' and 'to' and set 'aryAngles' with correct values
    if ((from >= 0) && (from < N_ANGLES) && (to >= 0) && (to < N_ANGLES)) {
      if (to >= from) {
        for (ix = from; ix <= to; ix++) {
          aryAngles[ix] = true;
        }
      }
      else {
        syntax_error = true;
        break;
      }
    }
    else {
      syntax_error = true;
      break;
    }
    from = 0;
    to = 0;
    doing_from_to = 0;
  }  // do
  while (arg != NULL);
  if (n_groups == 0)
    syntax_error = true;

  // Handle syntax errors
  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax"));
    Serial.println(F("  Example: SetAngle 0, 15-30, 45-50, 10"));
    Serial.println(F("  Example: SetAngle 0-359 to show all angles."));
    Serial.println(F("Notes: Use a space after each comma"));
    Serial.println(F("       No particular order is required"));
    Serial.println(F("       In a from-to pair, the 1st value must be lowest. From-to pairs can overlap ranges."));
  }
  else {                                                  // no errors detected, display the angles and start
    // We're ready to process multiple angles
    Serial.println(F(""));
    Serial.print(F("Angles:"));
    for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
      if (aryAngles[ix]) {
        Serial.print(ix, DEC);
        Serial.print(F(","));
      }
    }
    Serial.println(F(""));
    showDist();
  }  // if not (syntax_error)
}

void motorOff() {
  motor_enable = false;
  //Timer3.pwm(motor_pwm_pin, 0);
   analogWrite(motor_pwm_pin, 0);
  Serial.println(F(" "));
  Serial.println(F("Motor off"));
}

void motorOn() {
  motor_enable = true;
  //Timer3.pwm(motor_pwm_pin, pwm_val);
   analogWrite(motor_pwm_pin, pwm_val);
  rpm_err = 0;  // reset rpm error
  Serial.println(F(" "));
  Serial.println(F("Motor on"));
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
    if ((motor_rpm < rpm_min or motor_rpm > rpm_max) and pwm_val > 1000) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
      motorOff();
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    motor_check_timer = millis();
  }
}
void showPoints() {
  show_points = true;
  Serial.println(F(" "));
  Serial.println(F("Showing cartesian points"));
}
void hidePoints() {
  show_points = false;
  Serial.println(F(" "));
  Serial.println(F("Hiding cartesian points"));
}

void hideRaw() {
  raw_data = false;
  //Serial.println(F(" "));
  //Serial.println(F("Raw lidar data disabled"));
}

void showRaw() {
  raw_data = true;
  hideDist();
  hideRPM();
  //Serial.println(F(" "));
  //Serial.println(F("Lidar data enabled"));
}

void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
    if (sVal < rpm_min) {
      sVal = rpm_min;
      Serial.println(F(" "));
      Serial.print(F("RPM too low. Setting to minimum "));
      Serial.println(rpm_min);
    }
    if (sVal > rpm_max) {
      sVal = rpm_max;
      Serial.println(F(" "));
      Serial.print(F("RPM too high. Setting to maximum "));
      Serial.println(rpm_max);
    }
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetRPM 200"));
  }
  else {
    Serial.print(F("Old RPM setpoint:"));
    Serial.println(rpm_setpoint);
    rpm_setpoint = sVal;
    //Serial.println(F(" "));
    Serial.print(F("New RPM setpoint: "));
    Serial.println(sVal);
  }
}

void setKp() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKp 1.0"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kp to: "));
    Serial.println(sVal);
    Kp = sVal;
    rpmPID.SetTunings(Kp, Ki, Kd);
  }
}

void setKi() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKi 0.5"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Ki to: "));
    Serial.println(sVal);
    Ki = sVal;
    rpmPID.SetTunings(Kp, Ki, Kd);
  }
}

void setKd() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKd 0.001"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kd to: "));
    Serial.println(sVal);
    Kd = sVal;
    rpmPID.SetTunings(Kp, Ki, Kd);
  }
}

void setSampleTime() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);    // Converts a char string to an integer
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetSampleTime 20"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Sample time to: "));
    Serial.println(sVal);
    sample_time = sVal;
    rpmPID.SetSampleTime(sample_time);
  }
}



