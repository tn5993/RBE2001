#include <Servo.h>
#include <Encoder.h>
#include <BluetoothClient.h>
#include <BluetoothMaster.h>
#include <ReactorProtocol.h>
#include <TimerOne.h>

#define BLACK HIGH
#define WHITE LOW
#define thisROBOT 20  //0x14 10100
#define Low true
#define High false
#define LEFT true
#define RIGHT false
#define FORWARD true
#define BACKWARD false

enum facing { NORTH, EAST, SOUTH, WEST };
enum armPivotPosition { VERTICAL, HORIZONTAL };
enum armHeightPosition { TOP, MID, BOTTOM };

ReactorProtocol pcol(byte(thisROBOT));         // instantiate the protocol object and set the robot/team source address
BluetoothClient bt;                            // instantiate a Bluetooth client object
BluetoothMaster btmaster;                      // ...and a master object

byte pkt[10];                               
int sz;
byte data1[3];
byte type;
byte storageData, supplyData;
boolean isTwoSecond;
boolean shouldMove = true;

Servo portMotor, starboardMotor, armHeightMotor, armPivotMotor, clawMotor, storageMotor;
int portSensor = A0, midSensor = A1, starboardSensor = A2, portTurnSensor = 2, sbTurnSensor = 3;
int portBackSensor = A3, midBackSensor = A4, starboardBackSensor = A5;
int armPivotPOT = A6;
Encoder armHeightEncoder(A7, A8);
int armHeightStopButton = 18, pollSwitch = 19;

int I = 0, prev_error = 0;
int Kp =2, Ki = 0.08, Kd = 1;
int portprev = WHITE, sbprev = WHITE;
long sbCountBlack = 0, portCountBlack = 0;
long sbCountWhite = 0, portCountWhite = 0;
int currentX, currentY;
int BTstate = 1;
boolean sendHB = false;
facing facingDirection;
armPivotPosition armPivotPos;
armHeightPosition armHeightPos;

int rodStoredSlot = 3;
int rodTotal = 0;
void setup() {
  portMotor.attach(11);
  starboardMotor.attach(12);
  armPivotMotor.attach(10);
  armHeightMotor.attach(9);
  storageMotor.attach(8);
  clawMotor.attach(7);
  
  pinMode(portSensor, INPUT);
  pinMode(midSensor, INPUT);
  pinMode(starboardSensor, INPUT);
  pinMode(portTurnSensor, INPUT);
  pinMode(sbTurnSensor, INPUT);
  pinMode(portBackSensor, INPUT);
  pinMode(midBackSensor, INPUT);
  pinMode(starboardBackSensor, INPUT);
  pinMode(armPivotPOT, INPUT);
  pinMode(armHeightStopButton, INPUT);
  pinMode(pollSwitch, INPUT);

  attachInterrupt(0, doPortTurnSensorInterrupt, CHANGE);
  attachInterrupt(1, dosbTurnSensorInterrupt, CHANGE);

  Timer1.initialize(2000000);
  Timer1.attachInterrupt(timerISR);
  facingDirection = SOUTH;
  armPivotPos = VERTICAL;
  armHeightPos = TOP;
  
  currentX = 1; 
  currentY = 0;
  Serial.begin(9600);
}

boolean startButtonHit = false;

void loop() {
   processMessages();
}


/*****************************
  GRID ALGORITHM
******************************/
void go(int toY, int toX) {
    
  /* special cases */
  if ((currentX == 0 && facingDirection == WEST)) {
     backup_180(); forward(1); currentX = 1;
     facingDirection = EAST;
  } 
  else if (currentX == 2 && facingDirection == EAST) {
     backup_180(); forward(1); currentX = 1;
     facingDirection = WEST;
  }
  else if (currentY == 0 && facingDirection == SOUTH) {
     backup_180(); forward(1); currentY = 1;
     facingDirection = NORTH;
  } else if (currentY == 5 && facingDirection == NORTH) {
     backup_180(); forward(1); currentY = 4; 
     facingDirection = SOUTH;
  }
 
  int xDiff = toX - currentX;
  int yDiff = toY - currentY;
  
  goY(yDiff); // go on the y coordinate first
  goX(xDiff); // then go on the x coordinate

  
  /* update our position */
  currentX = toX;
  currentY = toY;
  Serial.print(facingDirection);
  Serial.print(", ");
  Serial.println(currentY);
  
}

void goX (int xDiff) {
  /* handle x coordinate movement */
  if (xDiff > 0) {
    switch (facingDirection) {
      case NORTH:
        turn_90(RIGHT);
        break;
      case SOUTH:
        turn_90(LEFT);
        break;
    }
    
    facingDirection = EAST;
    forward(abs(xDiff));
  } else if (xDiff < 0) {
    switch(facingDirection) {
      case NORTH:
        turn_90(LEFT);
        break;
      case SOUTH:
        turn_90(RIGHT);
        break;
    }
    
    facingDirection = WEST;
    forward(abs(xDiff));
  } else {
    Stop();
  }
}

void goY(int yDiff) {
  if (yDiff > 0) {
    switch(facingDirection) {
      case WEST:
        turn_90(RIGHT);
        break;
      case EAST:
        turn_90(LEFT);
        break;
    }
    
    facingDirection = NORTH;
    forward(abs(yDiff));
  } else if (yDiff < 0) {
    switch(facingDirection) {
      case WEST:
        turn_90(LEFT);     
        break;
      case EAST:
        turn_90(RIGHT);
        break;
    }
    
    facingDirection = SOUTH;  
    forward(abs(yDiff));
  } else {
    Stop();
  }
  
  
}

/************
  ROBOT MOVEMENTS
*/
/* move straight forward until reaching a number of intersection*/
void forward(int numIntersection) {
  
  long portCount = copyValue(portCountBlack);
  long sbCount = copyValue(sbCountBlack);
  
  long portGoal = portCount + numIntersection;
  long sbGoal = sbCount + numIntersection;
  
  while(portCount < portGoal || sbCount < sbGoal ) { //while two line sensors is not on the black line
    PIDGo(340);    
    portCount = copyValue(portCountBlack);
    sbCount = copyValue(sbCountBlack);
  }
  
  Stop();
}

/* move straight backward until reaching a number of intersection */
void back_up(int numIntersection) {
  
  long portCount = copyValue(portCountBlack);
  long sbCount = copyValue(sbCountBlack);
  
  long portGoal = portCount + numIntersection;
  long sbGoal = sbCount + numIntersection;
  
  while(portCount < portGoal || sbCount < sbGoal ) {
    Go(52, 70);
    portCount = copyValue(portCountBlack);
    sbCount = copyValue(sbCountBlack);
  }
  
  Stop();
}

/* turn 90 degree left or right */
void turn_90(boolean isLeft) {
  
  if (isLeft) { Go(50, 120); } 
  else { Go(110, 70); }
  
  delay(500);
  
  while (analogRead(midSensor) < 800) { // while mid sensor have not seen back line
       if (isLeft) {
         Go(50, 120); 
       } else {
         Go(110, 70); 
       }
  }
  
  Stop();
}

/* back up a little and turn 180 degree */
void backup_180(){
  Go(56, 70);
  delay(1600);
  turn_90(LEFT);
}

void Stop() {
 Go(90, 90); 
}

/* move straight forward until poll switch is hit */
void forwardUntilSwitch() {
  while(digitalRead(pollSwitch) == 1) {
    PIDGo(340);
  }
  Stop();
}

/**
	Adjust port and starboard motors accordingly base on the PID value.
	For example, the PID result can be in the range -180 < PID < 180
	if -180< PID < 0 means the right sensor or middle sensor is on the line
	else if 0 < PID < 180 means the left or middle sensor is on the line
	PID = 0 means the middle sensor is on the line (but this is rarely the case)
	
	Params:
		PID the pid value
		inMin: minimum PID value; this is sometime equal to PID_calc() outMin value
		inMax: maximum PID value; this is sometime equal to PID_calc() outMax value
*/

void PIDGo(int setpoint) {
   int sensors[3] = {portSensor, midSensor, starboardSensor}; 
   //int sensors[3] = {portBackSensor, midBackSensor, starboardBackSensor};
   const int max_speed = 120;
   const int min_speed = 90;
   
   int PID = PID_calc(setpoint, getPosition(sensors, 3), -180, 180);
   
   int port_speed = max_speed;
   int sb_speed = max_speed;
   
   if (PID < 0) { // we are on the right sensor
    port_speed = max_speed;
    sb_speed = map(PID, -180, 0, min_speed, max_speed); ;
   } else {
    port_speed = map(PID, 180, 0, min_speed, max_speed);
    sb_speed = max_speed;
   }
   
   Go(port_speed,sb_speed);
}

/* set port and starboard motors speed to move */
void Go(int pVelocity, int sbVelocity) {
 portGo(pVelocity);
 starboardGo(sbVelocity);
}

/* port motor rotate clockwise or counterclockwise*/
void portGo(int velocity) {
  portMotor.write(180-velocity);
}

/* starBoard motor rotate clockwise or counterclockwise */
void starboardGo(int velocity) {
  starboardMotor.write(velocity);
}

/************************
  ROBOT ARM CONTROL
*************************/
/* pick up a vertical rod */
void pickUpVerticalRod() {
  armPivotMove(VERTICAL); //go to vertical position first
  armHeightMove(TOP);
  clawOpen();
  armHeightMove(MID); //move arm to the position we want
  clawClose();
  delay(500);
  armHeightMove(TOP); //go to reset position
  armPivotMove(HORIZONTAL);
}

/* drop a rod vertically */
void dropVerticalRod() {
  armPivotMove(VERTICAL);
  armHeightMove(MID);
  clawOpen();
  delay(500);
  armHeightMove(TOP); //go to reset position
  armPivotMove(HORIZONTAL);
}

/* retrieve a horizontal rod*/
void retrieveHorizontalRod() {
  armPivotMove(HORIZONTAL); 
  clawOpen();
  armHeightMove(BOTTOM);
  clawClose();
  delay(500);
  armHeightMove(TOP); //go to reset position
}

/* insert a horizontal rod*/
void insertHorizontalRod() {
  armPivotMove(HORIZONTAL);
  armHeightMove(BOTTOM);
  clawOpen();
  delay(500);
  armHeightMove(TOP);//go to reset position
}

/********
	ARM HEIGHT CONTROL
*/
/* control the height of the arm holding the claw */
void armHeightMove(int pos) {
  switch (pos) {
    case TOP: //this is the highest position in the vertical perspective
      while (digitalRead(armHeightStopButton) == 1) { //If armHeightStopButton was hit when the arm was retrieving, then stop
        armHeightGo(50);
        delay(15);
      }
      armHeightEncoder.write(0); //reset the ecoder position to 0
      break;
    case MID:
      armHeightMove(TOP); //go to highest position and reset encoder
      while (armHeightEncoder.read() < 590) { //If reach position 590, then stop
         armHeightGo(110);
      }
      break;
    case BOTTOM:
      armHeightMove(TOP); //go to highest position and reset encoder
      while (armHeightEncoder.read() < 1070) { //If reach position 700, then stop
        armHeightGo(110);
      }
      break;
    default:
      break;
  }
  armHeightStop();
}

/* control the speed of the arm height */
void armHeightGo(int velocity) {
  armHeightMotor.write(180-velocity);
}

/* stop arm height*/
void armHeightStop() {
 armHeightGo(90); 
}

/****************************
	ARM PIVOT CONTROL
*****/

/* control the arm which allow transition from vertical to horizontal position */
void armPivotMove(int pos) {
  switch(pos) {
    case VERTICAL:
      while(analogRead(armPivotPOT) > 620) {
        armPivotGo(60);
      }
      break;
    case HORIZONTAL:
      while(analogRead(armPivotPOT) < 800) {
        armPivotGo(115);
      }
      break;
  }
  armPivotStop();
}

/* control arm pivot speed */
void armPivotGo(int velocity) {
  armPivotMotor.write(180-velocity);
}

/* stop arm pivot */
void armPivotStop() {
 armPivotGo(90); 
}

/***********
 CLAW CONTROL
*/

/* open claw */
void clawOpen() {
  int clawOpen= 0;
  clawGo(clawOpen);
}

/* stop claw */
void clawClose() {
  int clawClose = 180;
  clawGo(clawClose);
}

/* control arm position */
void clawGo(int pos) {
  clawMotor.write(pos);
}

/**** 
  ROBOT STORAGE CONTROL
*/
/**
	Retrieve and Store rod into robot storage
	param pos the position of the rod (HORIZONTAL or VERTICAL)
*/
void storeRod(int pos) {
    if (rodStoredSlot >= 1) {
      armPivotGo(HORIZONTAL);
    
      if (pos == HORIZONTAL) {
          retrieveHorizontalRod();
      } else if (pos == VERTICAL) {
          pickUpVerticalRod();
      }
      back_up(1);
      storageMoveOut(rodStoredSlot);
      dropVerticalRod();
      storageMoveOut(0);//reset
      rodStoredSlot -= 1;
    }

}

/**
	Extract rod from robot's storage and return it to storage tubes
	param pos the position of storage tubes (HORIZONTAL or VERTICAL)
*/
void retrieveRodAndReturn(int pos){
  if  (rodStoredSlot <= 3 && rodStoredSlot >= 0) {
    storageMoveOut(rodStoredSlot);
    pickUpVerticalRod();
    delay(1000);
    storageMoveOut(0); // reset
    rodStoredSlot += 1;
    
    if (pos == HORIZONTAL) {
        insertHorizontalRod();
    } else if (pos == VERTICAL) {
        dropVerticalRod();
    }
  }
}

/**
	Control robot storage location
	param slotNumber the slot number that the storage should extract to
*/
void storageMoveOut(int slotNumber) {
  switch (slotNumber) {
    case 0: // this is the reset position when the robot retract the robot into itself
      for (int i = 0; i < 80; i++) {
         storageMotorGo(i);
      }
      break;
    case 1: 
      storageMoveOut(0); //reset
      storageMotorGo(0); //go to the first position
      break;
    case 2: // slot 2
      storageMoveOut(0); //reset
      storageMotorGo(10); //go to the second position
      break;
    case 3: //slot 3
      storageMoveOut(0); //reset
      storageMotorGo(20); //go to the third position
      break;
    default:
      break;
  }
  armHeightStop();
}

void storageMotorGo(int velocity) {
  storageMotor.write(velocity);
}

/** 
	The function will identify the current position of the robot 
	relative to the line using weighted mean /sum
	For example: the below function will give the following result
		200 < x < 340 : if the left sensor is on the line
		x ~ 340: if the center sensor is on the line
		340< x < 450: if the right sensor is on the line
	The position function is helpful when calculating PID, for instance, we can
	set the setPoint to 340
	
	Output: the current position of the robot using 3 middle sensors
*/
long getPosition(int sensors[], int numSensors) {
  long sum = 0;
  long weightedsum = 0;
  for (int i = 0; i < numSensors; i++) {
    long value = analogRead(sensors[i]);
    sum += value;
    weightedsum += value * (i+1) * 180;
  }   
              
  return weightedsum/sum;            
}


/******************************************************************
  UTILITY FUNCTIONS
*/

/* count black and white encounters every interrupt on the port turn sensor */
void doPortTurnSensorInterrupt() {
  int readValue = digitalRead(portTurnSensor);
  if (readValue != portprev) {
    if (readValue == BLACK) {
      portCountBlack++;
    } else if (readValue == WHITE){
      portCountWhite++; 
    }
  }
  
  portprev = readValue;
}

/* count black and white encounters every interrupt on the starboard turn sensor */
void dosbTurnSensorInterrupt() {
  int readValue = digitalRead(sbTurnSensor);
  if (readValue != sbprev && readValue == BLACK) {
    if (readValue == BLACK) {
      sbCountBlack++;
    } else if (readValue == WHITE){
      sbCountWhite++; 
    }
  }
  
  sbprev = readValue;
}

/** 
	this function help to safely copy a value by stoping interrupt during
	the copy process
*/
long copyValue(long value) {
  long copyValue =0;
  noInterrupts();
  copyValue = value;
  interrupts();
  
  return copyValue;
}

/**
	This function is used for PID calculation
	Params:
		setpoint	the desired position
		currentPos 	the current position
		outMax		the maximum output
		outMin		the minimum output
	
	Output:
		PID value

*/
int PID_calc(int setpoint, int currentPos, int outMin, int outMax) {
  int error = setpoint - currentPos;
  int P = Kp * error;
  int D = Kd * (error-prev_error);
  I += Ki * error;
  
  if (I > outMax) { I = outMax; } 
  else if (I < outMin) { I = outMin; }
  
  prev_error = error;
  int PID = P + I + D;
  
  if (PID > outMax) { PID = outMax; }
  else if (PID < outMin) { PID = outMin;}
  
  return PID;
}

/*************************
  BLUETOOTH THINGY

**************************/
void sendHb() {
  pcol.setDst(0x00);			       // this will be a broadcast message --> destination address (control)
  sz = pcol.createPkt(0x07, data1, pkt);     // create a packet using the heartbeat type ID (there is no data)
  btmaster.sendPkt(pkt, sz);                 // send to the field computer 
}

void sendRadAlert(){
  delay(100);
  pcol.setDst(0x00);			       // this will be a broadcast message --> destination address (control)
  data1[0] = rodTotal == 1? 0x2C : 0xFF;                           // indicate a new fuel rod
  sz = pcol.createPkt(0x03, data1, pkt);     // create a packet using the radiation alert type ID (1 byte of data used this time)
  btmaster.sendPkt(pkt, sz);                 // send to the field computer
}

void sendHbAndRadAlert() {
   sendHb();
   sendRadAlert();
}

/* This is the main function that does all the magic things */
void processMessages() {
  if (btmaster.readPacket(pkt)) {// if we have received a message
    if (pcol.getData(pkt, data1, type)) { 
      boolean isSupply = false;
      boolean isStorage = false;     // see if we can extract the type and data
      switch (type) {                          // process the message based on the type
        case 0x01:                               // received a storage tube message
          storageData = data1[0];    
          isStorage = true;          // extract and save the storage-related data (the byte bitmask)
          break;
        case 0x02:                               // received a supply tube message
          supplyData = data1[0];
          isSupply = true;          // extract and save the supply-related data (the byte bitmask)
          break;
        default:                                 // ignore other types of messages
          break;
      }
      
   switch(BTstate) {
     case 1:
       if (isSupply) { // if supply rods message
         forward(1); //go to first position which is (0, 1)
          forwardUntilSwitch();
          storeRod(VERTICAL);
          rodTotal += 1;
      
      
        if (supplyData & 0x01) { // if has rod, go there
           go(1,2);
           forwardUntilSwitch();
           storeRod(HORIZONTAL);
           rodTotal += 1;
        }
       
        if (supplyData & 0x02) { // if has rod, go there
           go(2,2);
           forwardUntilSwitch();
           storeRod(HORIZONTAL);
           rodTotal += 1;
        }
      
        if (supplyData & 0x04) { // if has rod, go there
           go(3,2);
           forwardUntilSwitch();
           storeRod(HORIZONTAL);
           rodTotal += 1;
        }
      
        if (supplyData & 0x08) { // if has rod, go there
           go(4,2);
           forwardUntilSwitch();
           storeRod(HORIZONTAL);
           rodTotal += 1;
        }
      
        go(5,1); // this is the spent rod 2
        forwardUntilSwitch();
        pickUpVerticalRod();
        rodTotal += 1;
        
        BTstate = 2;
      }
        

      
      break;
    case 2:
      if (isStorage) { //if this is a storage message
        go(0, 1);
        forwardUntilSwitch();
        dropVerticalRod();
        rodTotal -= 1;
      
        if (!(storageData & 0x01)) { // if does not have rod, go there
          go(4,0);
          forwardUntilSwitch();
          retrieveRodAndReturn(HORIZONTAL);
          rodTotal -= 1;
        }
      
        if (!(storageData & 0x02)) { // if does not have rod, go there
          go(3,0);
          forwardUntilSwitch();
          retrieveRodAndReturn(HORIZONTAL);
          rodTotal -= 1;
        }
      
        if (!(storageData & 0x04)) { // if does not have rod, go there
          go(2,0);
          forwardUntilSwitch();
          retrieveRodAndReturn(HORIZONTAL);
          rodTotal -= 1;
        }
      
        if (!(storageData & 0x08)) { // if does not have rod, go there
          go(1,0); 
          forwardUntilSwitch();
          retrieveRodAndReturn(HORIZONTAL);
          rodTotal -= 1;
        }
      
        go(5,1); // this is the spent rod 2
        forwardUntilSwitch();
        retrieveRodAndReturn(VERTICAL);
        rodTotal -= 1;
       }
      break;
    }
  }
  }
}

void timerISR() {
  sendHbAndRadAlert();
}


