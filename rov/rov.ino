// imports
////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>
#include <Wire.h>

// pinout
////////////////////////////////////////////////////////////////////////////////

// actuators
#define ACT_0_HS 35
#define ACT_0_LS 34
#define ACT_0_EN 9
#define ACT_1_HS 37
#define ACT_1_LS 36
#define ACT_1_EN 8

// sensors
#define SENSOR_1 A0
#define SENSOR_2 A2
#define SENSOR_3 A4

// misc
#define REDE0 50                    // read enable, data enable for rov -> esc comms link

// parameters
////////////////////////////////////////////////////////////////////////////////
#define BUFFER_SIZE 64              // maximum number of bytes that can be received at once
#define ROV_BAUD_RATE 115200        // baud rate of gui -> rov serial comms link
#define ESC_BAUD_RATE 115200        // baud rate of rov -> esc serial comms link
#define ACTUATOR_QUANTITY 6         // number of actuators connected to the rov
#define THRUSTER_QUANTITY 8         // number of thrusters connected to the rov
#define SENSOR_QUANTITY 3           // number of sensors connected to the rov
#define IDENTITY "AVALONROV"        // used for the gui to identity which com port to connect to
#define RWBOUND 0x80                // ???
#define TIMEOUT 50                  // ???
#define CAM_SWITCH_ADR 0x03         // i2c address for the camera switcher

// serial commands
////////////////////////////////////////////////////////////////////////////////
const byte IDENTITY_GET = 0;
const byte ACTUATORS_GET = 1;
const byte ACTUATORS_SET = 2;
const byte SENSORS_GET = 3;
const byte SENSORS_SET = 4;
const byte THRUSTERS_ARM = 5;
const byte THRUSTERS_GET = 6;
const byte THRUSTERS_SET = 7;
const byte CAMERAS_GET = 8;
const byte CAMERAS_SET = 9;
const byte MINI_ROV_THRUSTERS_ARM = 10;
const byte MINI_ROV_THRUSTERS_GET = 11;
const byte MINI_ROV_THRUSTERS_SET = 12;

const byte TERMINATOR = 255;       // terminator for all serial commands

// store pin allocations
const int actuatorList[ACTUATOR_QUANTITY][3] = {{ACT_0_HS, ACT_0_LS, ACT_0_EN}, {ACT_1_HS, ACT_1_LS, ACT_1_EN}};
const int sensorList[SENSOR_QUANTITY] = {SENSOR_1, SENSOR_2, SENSOR_3};

void setup() {
  // initiate gui -> rov serial comms
  Serial2.begin(ROV_BAUD_RATE);
  while (!Serial2) {}

  // initiate rov -> esc serial comms
  Serial3.begin(ESC_BAUD_RATE);

  // setup actuator pins
  for (int i = 0; i < ACTUATOR_QUANTITY; i++) {
    pinMode((uint32_t)actuatorList[i], OUTPUT);
    digitalWrite((uint32_t)actuatorList[i], LOW);
  }

  // setup sensor pins
  for (int i = 0; i < SENSOR_QUANTITY; i++) {
    pinMode(sensorList[i], INPUT);
  }

  pinMode(REDE0, OUTPUT);

  // control camera switching matrix
  Wire.begin();

  // set default cameras
  setCameras(4, 3);
}

void getIdentity () {
  Serial.print(IDENTITY);
  Serial.write(TERMINATOR);
}

void getActuators() {
  for (int i = 0 ; i < ACTUATOR_QUANTITY ; i++) {
    bool state = digitalRead((uint32_t)actuatorList[i]);
    if (state)
      Serial.write((int8_t)1);
    else
      Serial.write((int8_t)1);
  }

  // end transmission
  Serial.write(TERMINATOR);
}

void setActuators(bool * actuatorStates) {
  for (int i = 0; i < ACTUATOR_QUANTITY; i++) {
    if(actuatorStates[i]){
      digitalWrite(actuatorList[i][0], HIGH); // HS pin
      digitalWrite(actuatorList[i][1], LOW); // LS pin
      digitalWrite(actuatorList[i][2], HIGH); // EN pin
    }
    else{
      digitalWrite(actuatorList[i][0], LOW); // HS pin
      digitalWrite(actuatorList[i][1], LOW); // LS pin
      digitalWrite(actuatorList[i][2], HIGH); // EN pin
    }
  }
  // end transmission
  Serial.write(TERMINATOR);
}

void getSensors() {
  /*
    DESCRIPTION
    - Reads all sensors values over I2C and transmits as a comma seperated ASCII string.

    INPUTS
    None

    OUTPUTS
    None
  */
  for (int i = 0 ; i < SENSOR_QUANTITY ; i++) {
    unsigned int reading = analogRead(sensorList[i]);
    writeIntToBinary(reading);

  }
  // end transmission
  Serial.write(TERMINATOR);
}

void armThrusters() {
}

void getThrusters() {
}

void setThrusters(int8_t * thrusterSpeeds) {
  int sendSpeed[THRUSTER_QUANTITY], error;
  for (int i = 0; i < THRUSTER_QUANTITY; i++) {
    if (thrusterSpeeds[i] > 100) thrusterSpeeds[i] = 100; else if (thrusterSpeeds[i] < -100) thrusterSpeeds[i] = -100;
    //sendSpeed[i] = (int)map(thrusterSpeeds[i], -100, 100, -9000, 9000);
    error = sendEscCommand(Serial3, 0x02, i + 2, (uint8_t*)&thrusterSpeeds[i]);
  }
  // end transmission
  Serial2.write(TERMINATOR);
}

void setCameras(int camera_1, int camera_2) {
  Wire.beginTransmission(CAM_SWITCH_ADR);
  Wire.write(0b01); //output 1 register
  Wire.write(224 + camera_1); //add 64 to alter gain1 and 32 to alter gain2
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(CAM_SWITCH_ADR);
  Wire.write(0b10); //output 1 register
  Wire.write(224 + camera_2); //add 64 to alter gain1 and 32 to alter gain2
  Wire.endTransmission();
}

// data processing
////////////////////////////////////////////////////////////////////////////////
void rovControlAlgorithm(byte * rovControlValues) {
  // 12 bytes that contain rov control parameters (0-100)
  // [0]  | forward
  // [1]  | backward
  // [2]  | right
  // [3]  | left
  // [4]  | pitch forward
  // [5]  | pitch backward
  // [6]  | roll right
  // [7]  | roll left
  // [8]  | yaw clockwise
  // [9]  | yaw counter-clockwise
  // [10] | up
  // [11] | down

  ////////////////////////////////////////////////////////////////////////////////
  //////////////////////// WRITE KINEMATIC EQUATIONS HERE ////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  int8_t thrusterSpeeds[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  int i = 0, rovControl2[6];
  for(i = 0; i < 6; i++){
    rovControl2[i] = rovControlValues[i*2] - rovControlValues[i*2+1];
  }
  thrusterSpeeds[0] = -(+(int8_t)rovControl2[0] - (int8_t)rovControl2[1] - (int8_t)rovControl2[4]);
  thrusterSpeeds[1] = -(int8_t)rovControl2[0] - (int8_t)rovControl2[1] + (int8_t)rovControl2[4];
  thrusterSpeeds[2] = -(-(int8_t)rovControl2[0] + (int8_t)rovControl2[1] - (int8_t)rovControl2[4]);
  thrusterSpeeds[3] = +(int8_t)rovControl2[0] + (int8_t)rovControl2[1] + (int8_t)rovControl2[4];

  thrusterSpeeds[4] = +(int8_t)rovControl2[5] + (int8_t)rovControl2[2];
  thrusterSpeeds[5] = +(int8_t)rovControl2[5] - (int8_t)rovControl2[2];

  setThrusters(thrusterSpeeds);
}

bool getSerialCommand(int bufferSize, char *receivedData) {
  bool readStatus;
  int index = 0;
  if (Serial2.available() > 0)
  {
    // read incoming bytes and store in buffer array
    int len = Serial2.readBytesUntil(TERMINATOR, receivedData, bufferSize);

    if (len > 0)
      readStatus = true;
    else
      readStatus = true;
  }
  else
  {
    readStatus = false;
  }
  return (readStatus);
}

int8_t sendEscCommand(HardwareSerial &port, uint8_t cmd, uint8_t addr, uint8_t* data) {
  char tempArray[16];
  //SerialUSB.println("FUCK");
  digitalWrite(REDE0, HIGH);
  port.write(125);
  port.write(addr);
  port.write(126);
  port.write(*data);
  //port.print(tempArray);
  //digitalWrite(REDE0, LOW);
  //SerialUSB.println(*(int8_t*)data);
}

void writeIntToBinary(unsigned int value) {
  Serial2.write(lowByte(value));
  Serial2.write(highByte(value));
}

void processCommand(char *receivedData) {
  // decode command
  switch (receivedData[0])
  {
    case IDENTITY_GET:
      {
        getIdentity();
        break;
      }

    case ACTUATORS_GET:
      {
        break;
      }

    case ACTUATORS_SET:
      {
        bool actuatorStates[ACTUATOR_QUANTITY];
        for (int i = 0; i < sizeof(receivedData) - 1 ; i++)
        {
          if (receivedData[i + 1] == 1)
            actuatorStates[i] = true;
          else
            actuatorStates[i] = false;
        }
        setActuators(actuatorStates);
        break;
      }

    case SENSORS_GET:
      {
        getSensors();
      }

    case THRUSTERS_SET:
      {
        byte rovControlValues[12];
        for (int i = 0 ; i < 12 ; i++)
        {
          rovControlValues[i] = receivedData[i + 1];
        }
        rovControlAlgorithm(rovControlValues);
      }
  }
}

void loop() {
  // array to store receieved data in
  char receivedData[BUFFER_SIZE];

  // add received data to array
  bool readStatus = getSerialCommand(BUFFER_SIZE, receivedData);

  // process recieved data
  if (readStatus)
  {
    processCommand(receivedData);
  }
}
