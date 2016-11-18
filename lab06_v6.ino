/* 
IRM Lab06 : Closed-loop control

*/

// Include the required libraries here
#include <string.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Wire.h>


// define your global variables here
// create a new object of Adafruit_MotorShield, you can call it AFMS
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// create one instance for each motor, call them motor1 and motor2
Adafruit_StepperMotor *motor1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);


// initialize pinouts for limit switches
int switch1 = 6;
int switch2 = 7;
int switch3 = 4;
int switch4 = 5;

// initialize step size (Number of steps in each iteration)
int stepSize = 20; 

// variables for motor position
int pos1 = 0;
int pos2 = 0;

// maximum settings for motors
float maxspeed = 600; // [steps/s]

// hardcoded path to follow
const int pathsize = 52;
int path[pathsize][3] = {
{0,	0,	maxspeed},
{-183,	0,	maxspeed},
{-453,	0,	maxspeed},
{-685,	0,	maxspeed},
{-954,	0,	maxspeed},
{-1199,	12,	maxspeed},
{-1419,	12,	maxspeed},
{-1664,	24,	maxspeed},
{-1908,	0,	maxspeed},
{-2067,	-98,	maxspeed},
{-2153,	-281,	maxspeed},
{-2165,	-489,	maxspeed},
{-2165,	-697,	maxspeed},
{-2092,	-893,	maxspeed},
{-1933,	-1028,	maxspeed},
{-1737,	-1125,	maxspeed},
{-1541,	-1223,	maxspeed},
{-1309,	-1333,	maxspeed},
{-1089,	-1407,	maxspeed},
{-856,	-1492,	maxspeed},
{-624,	-1578,	maxspeed},
{-440,	-1725,	maxspeed},
{-318,	-1896,	maxspeed},
{-245,	-2104,	maxspeed},
{-196,	-2336,	maxspeed},
{-183,	-2569,	maxspeed},
{-196,	-2801,	maxspeed},
{-208,	-3009,	maxspeed},
{-269,	-3229,	maxspeed},
{-391,	-3425,	maxspeed},
{-563,	-3535,	maxspeed},
{-795,	-3572,	maxspeed},
{-3119,	-3572,	maxspeed},
{-3315,	-3523,	maxspeed},
{-3450,	-3401,	maxspeed},
{-3511,	-3205,	maxspeed},
{-3511,	-2972,	maxspeed},
{-3425,	-2826,	maxspeed},
{-3266,	-2752,	maxspeed},
{-3034,	-2740,	maxspeed},
{-1664,	-2801,	maxspeed},
{-1443,	-2801,	maxspeed},
{-1272,	-2728,	maxspeed},
{-1211,	-2569,	maxspeed},
{-1235,	-2385,	maxspeed},
{-1358,	-2226,	maxspeed},
{-1566,	-2153,	maxspeed},
{-2997,	-1492,	maxspeed},
{-3180,	-1333,	maxspeed},
{-3315,	-1113,	maxspeed},
{-3388,	-820,	maxspeed},
{-3437,	171,	maxspeed}
};

// Functions

int sign(int x) {
  if (x>0) return 1;
  if (x<0) return -1;
  if (x==0) return 0;
}

void setup() {
  
  
  // Start the serial communication at 115200 baud rate
  Serial.begin(115200);
  
  // Set serial communication timeout
  Serial.setTimeout(10000); 
  
  // Start the Adafruit Motor Shield and set the maximum speed of the stepper
  AFMS.begin();
  motor1->setSpeed(120);
  motor2->setSpeed(120);
  // Set the input pins
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(switch4, INPUT);
  

}

int move_steps (int steps, int dir, int motor) {
  int switch_check;

  Adafruit_StepperMotor *myMotor;

  // Check the motor and direction of movement, and set the limiting switch accordingly
  if(motor == 1){
    myMotor = motor1;
    if(dir == FORWARD){
      switch_check = switch1;
    }else 
    if(dir == BACKWARD){
      switch_check = switch2;
    }
  }else 
  if(motor == 2){
    myMotor = motor2;
    if(dir == FORWARD){
      switch_check = switch3;
    }else 
    if(dir == BACKWARD){
      switch_check = switch4;
    }
  }
  
  // Limit the total number of steps to 999
  steps = constrain(steps,0,999);

  // Move the motor if steps > 0 and the limit switch has not been reached
  while(steps>0&&(digitalRead(switch_check))){
    myMotor->step(1,dir);
    steps--;
  }

  myMotor->release();
  
  // return ASCII for the switches
    if(!digitalRead(switch1)) return '1';
    if(!digitalRead(switch2)) return '2';
    if(!digitalRead(switch3)) return '3';
    if(!digitalRead(switch4)) return '4';
    return '0';
}

int move_to (int tar1, int tar2, float maxfreq) {
  // calculate minimal motor loop period
  unsigned long minperiod = 1E6/maxfreq; // [us]
  // calculate moving distance
  int dist1 = tar1 - pos1;
  int dist2 = tar2 - pos2;
  // active switches for both axis
  int sw1 = (dist1 > 0)?switch1:switch2;
  int sw2 = (dist2 > 0)?switch3:switch4;
  // timestamp
  unsigned long tic;
  // move motors
  if (dist1!=0 && dist2==0) {
    // move only motor1
    // motor loop: exit when reached target or switch activated
    while (pos1!=tar1 && digitalRead(sw1)) {
      tic = micros(); // update timestamp
      if (dist1 > 0) motor1->onestep(FORWARD, SINGLE);  // do motor step
      else motor1->onestep(BACKWARD, SINGLE);
      pos1+=sign(dist1);  // increment/decrement motor posititon
      delayMicroseconds(minperiod+tic<micros()?0:minperiod+tic-micros());  // limit loop frequency
    }
  } else if (dist1==0 && dist2!=0) {
    // move only motor2
    // motor loop: exit when reached target or switch activated
    while (pos2!=tar2 && digitalRead(sw2)) {
      tic = micros(); // update timestamp
      if (dist2 > 0) motor2->onestep(FORWARD, SINGLE);  // do motor step
      else motor2->onestep(BACKWARD, SINGLE);
      pos2+=sign(dist2);  // increment/decrement motor posititon
      delayMicroseconds(minperiod+tic<micros()?0:minperiod+tic-micros());  // limit loop frequency
    }
  } else if (dist1!=0 && dist2!=0) {
    // move both motors
    // init step counters
    unsigned int ctr1 = 0;
    unsigned int ctr2 = 0;
    // store abs values of distances
    unsigned int adist1 = abs(dist1);
    unsigned int adist2 = abs(dist2);
    // motor loop: exit when reached target or switch activated
    while (pos1!=tar1 && pos2!=tar2 && digitalRead(sw1) && digitalRead(sw2)) {
      tic = micros(); // update timestamp
      // check if can move motor1
      if (adist1*(ctr1+ctr2)>=(adist1+adist2)*ctr1) {
        if (dist1 > 0) motor1->onestep(FORWARD, SINGLE);  // do motor step
        else motor1->onestep(BACKWARD, SINGLE);
        pos1+=sign(dist1);  // increment/decrement motor posititon
        ctr1++;  // increment counter value
      }
      // check if can move motor2
      if (adist2*(ctr1+ctr2)>=(adist1+adist2)*ctr2) {
        if (dist2 > 0) motor2->onestep(FORWARD, SINGLE);  // do motor step
        else motor2->onestep(BACKWARD, SINGLE);
        pos2+=sign(dist2);  // increment/decrement motor posititon
        ctr2++;  // increment counter value
      }
      delayMicroseconds(minperiod+tic<micros()?0:minperiod+tic-micros());  // limit loop frequency
    }  // motor loop
  }  // move both motors
  // return ASCII for the switches
  if(!digitalRead(switch1)) return '1';
  if(!digitalRead(switch2)) return '2';
  if(!digitalRead(switch3)) return '3';
  if(!digitalRead(switch4)) return '4';
  return '0';
}


void loop() {
  
  // Initialize parameters
  char command[50];
  char check;
  byte read_check;

  // Check if there is a command on the serial port
  if (Serial.available() > 0)
  {
    read_check = Serial.readBytes(command,5);
    // If the command is not 5 bytes long, discard it
    if ((int)read_check == 5)
    {
      command[5] = '\0';
      check = '0';
    }
    else
    {      
      command[0] = '\0';
      check = '5';
      Serial.print(check);
    }
  }
  
  // Proper command contains 5 bytes:
  // 	First byte is the stage number: 1 or 2
  //	Second byte is the direction: 1 or 2
  //	Third - Fifth bytes are number of steps: 000 - 999
  if (command[0]!=0)
  {
    // Check the first byte and if it is not '1' or '2' discard it
    // First byte determines the stage (stepper motor) that needs to be moved
    if( command[0]!= '1' && command[0] != '2' && command[0] != 'S' && command[0] != 'R') {
      check = '5';
    }
    // Check the second byte and if it is not '1' or '2' discard it
    // Second byte determines the direction
    if( command[1]!= '1' && command[1] != '2') {
      check = '5';
    }	
    // Check that third to fifth bytes are between '0' and '9'
    // make sure to convert from chars to integers (subtract 48, the ASCII constant) and multiply accordingly
    for( int i = 2; i<5; i++){
      if(!(command[i] >= '0' && command[i] <= '9')) {
        check = '5';        
      }
    }

    // If everything is fine, move the motors
    if( check != '5'){
      if (command[0] == 'S') {
        // Configure TWI clock for fast mode
        REG_TWI0_CWGR = 0x6565;
        // start moving parcours 
        double coef = 1.04;
        for (int i = 0; i<pathsize; i++) {
          check = move_to(coef*path[i][0], coef*path[i][1], path[i][2]);
          if (check != '0') break;
        }
        motor1->release();
        motor2->release();
        // Configure TWI clock for normal mode
        REG_TWI0_CWGR = 0x1D0D0;
      } else if (command[0] == 'R') {
        // Configure TWI clock for fast mode
        REG_TWI0_CWGR = 0x6565;
        // move to starting point
        check = move_to(0, 0, maxspeed);
        motor1->release();
        motor2->release();
        // Configure TWI clock for normal mode
        REG_TWI0_CWGR = 0x1D0D0;
      } else {  
        // regular movement
        int steps = (command[4]-'0') + 10*(command[3]-'0') + 100*(command[2]-'0');
        check = move_steps (steps, command[1]-'0', command[0]-'0');
    }
  }
    // Check is sent over the serial back to microprocessor:
    //     '0' if motor moved 
    //     '1' if switch 1 is pressed
    //     '2' if switch 2 is pressed
    //     '3' if switch 3 is pressed
    //     '4' if switch 4 is pressed
    //     '5' if command is bad
    Serial.print(check);
    
    // Reset the command
    command[0] = '\0';
    Serial.flush();
  }
  
  // Delay in ms
  delay(5);
}

