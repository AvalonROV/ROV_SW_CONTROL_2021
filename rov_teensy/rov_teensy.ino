// imports
////////////////////////////////////////////////////////////////////////////////
//#include <Servo.h>

// pinout
////////////////////////////////////////////////////////////////////////////////

// actuators
#define ACT_1 11
#define ACT_2 11
#define ACT_3 8
#define ACT_4 9
#define ACT_5 6
#define ACT_6 7

// thrusters
#define THRUST_1 9
#define THRUST_2 9
#define THRUST_3 9
#define THRUST_4 9
#define THRUST_5 9
#define THRUST_6 9
#define THRUST_7 9
#define THRUST_8 9

// sensors
#define SENSOR_1 A0
#define SENSOR_2 A2
#define SENSOR_3 A4

// parameters
////////////////////////////////////////////////////////////////////////////////
#define BUFFER_SIZE 64
#define BAUD_RATE 115200
#define ACTUATOR_QUANTITY 6
#define THRUSTER_QUANTITY 8
#define SENSOR_QUANTITY 3
#define IDENTITY "AVALONROV"

// serial commands
////////////////////////////////////////////////////////////////////////////////

// rov commands
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

// mini rov commands
const byte MINI_ROV_THRUSTERS_ARM = 10;
const byte MINI_ROV_THRUSTERS_GET = 11;
const byte MINI_ROV_THRUSTERS_SET = 12;

const byte TERMINATOR = 255;

const int actuatorList[ACTUATOR_QUANTITY] = {ACT_1, ACT_2, ACT_3, ACT_4, ACT_5, ACT_6};
const int thrusterList[THRUSTER_QUANTITY] = {THRUST_1, THRUST_2, THRUST_3, THRUST_4, THRUST_5, THRUST_6, THRUST_7, THRUST_8};
const int sensorList[SENSOR_QUANTITY] = {SENSOR_1, SENSOR_2, SENSOR_3};
//const Servo thrusters[THRUSTER_QUANTITY];

void setup() {
  /*
    DESCRIPTION
    - Initiates serial communication interface and sets ups I/O.

    INPUTS
    None

    OUTPUTS
    None
  */
  Serial.begin(BAUD_RATE);
  while (!Serial) {}

  pinMode(13, OUTPUT);

  // SETUP ACTUATORS PINS
  for (int i = 0; i < ACTUATOR_QUANTITY; i++) {
    pinMode(actuatorList[i], OUTPUT);
    digitalWrite(actuatorList[i], LOW);
  }

  //SETUP THRUSTER PINS
  for (int i = 0; i < THRUSTER_QUANTITY; i++) {
    //thrusters[i].attach(thrusterList[i]);
    //thrusters[i].writeMicroseconds(1500);
  }
}

// get commands
////////////////////////////////////////////////////////////////////////////////
void armThrusters() {
  /*
    DESCRIPTION
    Arms the thruster ESCs by outputting the servo signal for neutral speed.

    INPUTS
    None

    OUTPUTS
    None
  */
  for (int i = 0; i < THRUSTER_QUANTITY; i++) {
    //thrusters[i].writeMicroseconds(1500);
  }
}

void getThrusters() {
  /*
    DESCRIPTION
    - Reads set speed of each thruster and transmits as a comma seperated ASCII string.

    INPUTS
    None

    OUTPUTS
    None
  */
}

void getActuators() {
  /*
    DESCRIPTION
    - Reads state of each actuator digital pin and transmits as a comma seperated ASCII string.

    INPUTS
    None

    OUTPUTS
    None
  */
  for (int i = 0 ; i < ACTUATOR_QUANTITY ; i++) {
    bool state = digitalRead(actuatorList[i]);
    Serial.print(state);
    if (i != SENSOR_QUANTITY - 1)
      Serial.print(TERMINATOR);
  }

  // end transmission
  Serial.println();
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
    uint8_t reading = analogRead(sensorList[i]);
    writeIntToBinary(reading);
    
  }
  // end transmission
  Serial.write(TERMINATOR);
}

// set commands
////////////////////////////////////////////////////////////////////////////////
void setThrusters(int *thrusterSpeeds) {
  /*
     DESCRIPTION
     - Sets the PWM for each thruster pin.

     INPUTS
     - thrusterSpeeds (int array) = Pointer to the array that stores the processed thruster speeds (0-1023).

     OUTPUTS
     None
  */
  for (int i = 0; i < THRUSTER_QUANTITY; i++) {
    //thrusters[i].writeMicroseconds(map(thrusterSpeeds[i], 1, 999, 1100, 1900));
  }
  // end transmission
  Serial.write(TERMINATOR);
}

void setActuators(bool * actuatorStates) {
  /*
     DESCRIPTION
     Sets the actuator pins HIGH or LOW.

     INPUTS
     actuatorStates (bool array) = Name of the array that stores the processed actuator states (true/false).

     OUTPUTS
     None
  */
  for (int i = 0; i < ACTUATOR_QUANTITY; i++) {
    digitalWrite(actuatorList[i], actuatorStates[i]);
  }
  // end transmission
  Serial.write(TERMINATOR);
}

void setCameras() {
  /*
    DESCRIPTION
    - Sets which analogue cameras are to be sent up the tether.

    INPUTS
    None

    OUTPUTS
    None
  */

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
  if(rovControlValues[0] > 50)
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

  int thrusterSpeeds[8] = {500, 500, 500, 500, 500, 500, 500, 500};
  setThrusters(thrusterSpeeds);
}

int charArrayToInt(const char *data, size_t start, size_t finish) {
  /*
    DESCRIPTION
    - Converts a specific region of a char array into an integer.
    - For example, it could return 56 from ['2','5','6','1'].

    INPUTS
    - data (char array) = Pointer to the char array where the data is stored.
    - start (int) = The index of the first desired character in the array.
    - finish (int) = The index of the last desired character in the array.

    OUTPUTS
    - val (int) = The integer value from the char array
  */
  int val = 0;
  while (start < finish) {
    val = val * 10 + data[start++] - '0';
  }
  return val;
}

bool getSerialCommand(int bufferSize, char *receivedData) {
  /*
    DESCRIPTION
    - Receives bytes of data over serial and stores them in an array

    INPUTS
    - bufferSize (int) = Maximum number of characters to be stored in the received data array.
    - receivedData (char array) = Name of the array to stores the received data in.

    OUTPUTS
    - readStatus (bool) = True if a command has been recieved, False otherwise.
  */
  bool readStatus;
  int index = 0;

  if (Serial.available() > 0)
  {
    // read incoming bytes and store in buffer array
    int len = Serial.readBytesUntil(255, receivedData, bufferSize);
    //while(Serial.available()>0){Serial.read(); Serial.flush();}
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

void writeIntToBinary(unsigned int value) {
  Serial.write(lowByte(value));
  Serial.write(highByte(value));
}

void processCommand(char *receivedData) {
  /*
    DESCRIPTION
    - Breaks down an ASCII string command to determine the required action, and called the neccessary function.

    INPUTS
    - receivedData (char array) = Name of the array that contains the received data.

    OUTPUTS
    None
  */
  // decode command
  switch (receivedData[0])
  {
    case IDENTITY_GET:
      {
        Serial.print(IDENTITY);
        Serial.write(TERMINATOR);
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

void loop()
{
  // array to store receieved data in
  char receivedData[BUFFER_SIZE];

  // add recieved data to array
  bool readStatus = getSerialCommand(BUFFER_SIZE, receivedData);

  // process recieved data
  if (readStatus)
  {
    processCommand(receivedData);
  }
}
