/**@main.ino */

// IMPORTS
#include <Servo.h>

// PINOUT

// PARAMETERS
#define BUFFER_SIZE 40
#define BAUD_RATE 115200
#define IDENTITY "AVALONSURFACE"

void setup() {
  /*
    PURPOSE
    Initiates serial communication interface and sets ups actuators/thruster.

    INPUT
    None

    RETURNS
    None
  */
  Serial.begin(BAUD_RATE);
  while (!Serial) {}
}

int charArrayToInt(const char *data, size_t start, size_t finish) {
  /*
    PURPOSE
    Converts a specific region of a char array into an integer.
    For example, it could return 56 from ['2','5','6','1'].

    INPUT
    const char *data = Pointer to the char array where the data is stored.
    size_t start = The index of the first desired character in the array.
    size_t finish = The index of the last desired character in the array.

    RETURNS
    val = The integer value from the char array
  */
  int val = 0;
  while (start < finish) {
    val = val * 10 + data[start++] - '0';
  }
  return val;
}

bool getSerialCommand(int bufferSize, char *receivedData) {
  /*
    PURPOSE
    Receives data over serial and stores it in a char array via a pointer.
    Keeps trying to receive data until a newline character is received.

    INPUTS
    int bufferSize = Maximum number of characters to be stored in the received data array.
    char *receivedData = Name of the array to stores the received data in.

    RETURNS
    readStatus = True if a command has been recieved, False otherwise.
  */
  bool readStatus;
  char character;
  int index = 0;

  if (Serial.available() > 0) {
    // KEEP READING UNTIL COMMAND IS FULLY RECEIVED
    while (true) {
      // READ SINGLE CHARACTER
      character = Serial.read();

      // IF COMMAND IS COMPLETE
      if (character == '\n') {
        // ADD NULL TERMINATION WHEN DATA IS FULLY RECEIVED
        receivedData[index] = '\0';
        readStatus = true;
        // STOP RECEIVING
        break;
      }
      // IF VALID CHARACTER IS RECEIVED
      else if (character != -1) {
        if (index < (bufferSize - 1)) {
          // ADD CHARACTER TO CHAR BUFFER
          receivedData[index] = character;
          index++;
        }
      }
    }
  }
  else {
    readStatus = false;
  }
  return (readStatus);
}

void processCommand(char *receivedData) {
  /*
    PURPOSE
    Breaks down an ASCII string command to determine the required action, and called the neccessary function.

    INPUT
    char *receivedData = Name of the array that contains the received data.

    RETURNS
    None
  */
  // REMOVE BLANK SPACES FROM ARRAY (FEATURE NOT IMPLEMENTED)


  // CHECK IF THE COMMAND IS VALID
  if (receivedData[0] == '?') {

    // SURFACE COMMAND
    if (receivedData[1] == 'R') {
      switch (receivedData[2]) {

        // INVALID COMMAND
        default:
          break;
      }
    }

    // IDENTITY REQUEST
    else if (receivedData[1] == 'I') {
      Serial.println(IDENTITY);
    }

    else {}
  }
  else {}
}

void loop() {
  /*
    PURPOSE
    Main execution loop.

    INPUT
    None

    RETURNS
    None
  */
  
  // DEFINE ARRAY TO STORE RECEIVED DATA IN
  char receivedData[BUFFER_SIZE];

  // UPDATE ARRAY WITH RECEIVED DATA
  bool readStatus = getSerialCommand(BUFFER_SIZE, receivedData);

  // COMMAND SUCCESSFULLY RECEIVED
  if (readStatus)
  {
    processCommand(receivedData);
  }
}
