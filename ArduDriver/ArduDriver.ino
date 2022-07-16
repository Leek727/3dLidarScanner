#include <SoftwareSerial.h> //header file of software serial port

SoftwareSerial Serial1(10, 11); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX

// ------------------------------ LIDAR VARS ------------------------------
int dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package


// ------------------------------ STEPPER VARS ------------------------------
// num of motors
#define num_steppers 2

// stepper motor pinout
int ids[num_steppers][4] = {
  //{10,11,12,13},
  {6, 7, 8, 9},
  {2, 3, 4, 5}
};

// track steps of individual stepper motor for phase power
int steps[num_steppers] = {0, 0};


// ------------------------------ SETUP ------------------------------
void setup() {
  Serial.begin(9600); //set bit rate of serial port connecting Arduino with computer
  while (Serial) {
    if (Serial.available() > 0) {
      String incomingString = Serial.readString();
      if (incomingString.substring(0, 9) == "connected") {
        break;
      }
      Serial.println(incomingString);
    }
  }


  Serial1.begin(115200); //set bit rate of serial port connecting LiDAR with Arduino

  // set all pins to output
  for (int i = 0; i < num_steppers; i++) {
    for (int j = 0; j < 4; j++) {
      pinMode(ids[i][j], OUTPUT);
    }
  }
}

// ------------------------------ MAIN ------------------------------
int steps_per_rev = 4076;
int basesteps = 2;
int numsteps = 2;
void loop() {
  // for each degree of base rotation - rotate 180 lidar
  for (int x = 0; x < steps_per_rev / basesteps; x++) {
    // 180 linear scan
    for (int j = 0; j < (steps_per_rev / 2) / numsteps; j++) {
      // read lidar
      readlidar(x, j);
 
     
      // move lidar one step
      for (int k = 0; k < numsteps; k++) {
        stepper(1, 0);
        delayMicroseconds(1500);
      }
    }



    // move lidar back to starting pos
    for (int j = 0; j < (steps_per_rev / 2) / numsteps; j++) {
      // move lidar one step
      for (int k = 0; k < numsteps; k++) {
        stepper(1, 1);
        delayMicroseconds(1500);
      }
    }

    // move base one step
    for (int j = 0; j < basesteps; j++) {
      stepper(0, 1);
      delayMicroseconds(1500);
    }

  }
  // move motor back
  for (int x = 0; x < steps_per_rev / basesteps; x++) {
    for (int j = 0; j < basesteps; j++) {
      stepper(0, 0);
      delayMicroseconds(1500);
    }
  }
  Serial.println("Done!");
  while (1) {
    ;
  }
}

void readlidar(int x, int y) {
  while (true) {
    if (Serial1.available()) { //check if serial port has data input
      if (Serial1.read() == HEADER) { //assess data package frame header 0x59
        uart[0] = HEADER;
        if (Serial1.read() == HEADER) { //assess data package frame header 0x59
          uart[1] = HEADER;
          for (i = 2; i < 9; i++) { //save data in array
            uart[i] = Serial1.read();
          }
          check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
          if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
            dist = uart[2] + uart[3] * 256; //calculate distance value
            strength = uart[4] + uart[5] * 256; //calculate signal strength value
            temprature = uart[6] + uart[7] * 256; //calculate chip temprature
            temprature = temprature / 8 - 256;

            Serial.print(x);
            Serial.print(',');
            Serial.print(y);
            Serial.print(',');
            Serial.print(dist); // cm
            Serial.print(',');
            Serial.print(strength); //output signal strength
            Serial.print(":\n");
            //Serial.println(temprature);
            break;
          }
        }
      }
    }
  }
}
void stepper(int stepper_id, int Direction) {
  for (int x = 0; x < 1; x++) {
    // stepper motor phases - input for dual hbridge
    switch (steps[stepper_id]) {
      case 0:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], HIGH);
        break;
      case 1:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], HIGH);
        digitalWrite(ids[stepper_id][3], HIGH);
        break;
      case 2:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], HIGH);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
      case 3:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], HIGH);
        digitalWrite(ids[stepper_id][2], HIGH);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
      case 4:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], HIGH);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
      case 5:
        digitalWrite(ids[stepper_id][0], HIGH);
        digitalWrite(ids[stepper_id][1], HIGH);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
      case 6:
        digitalWrite(ids[stepper_id][0], HIGH);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
      case 7:
        digitalWrite(ids[stepper_id][0], HIGH);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], HIGH);
        break;
      default:
        digitalWrite(ids[stepper_id][0], LOW);
        digitalWrite(ids[stepper_id][1], LOW);
        digitalWrite(ids[stepper_id][2], LOW);
        digitalWrite(ids[stepper_id][3], LOW);
        break;
    }

    // determine phase
    if (Direction == 1) {
      steps[stepper_id]++;
    }
    if (Direction == 0) {
      steps[stepper_id]--;
    }
    if (steps[stepper_id] > 7) {
      steps[stepper_id] = 0;
    }
    if (steps[stepper_id] < 0) {
      steps[stepper_id] = 7;
    }
  }
}
