/*
  Name:    setCurrent.ino
  Created: 19-08-2018
  Author:  SolidGeek
  Description: This is a very simple example of how to set the current for the motor
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VescUart.h>
#include <Wire.h>

#define SWITCH 10 // limit switch
#define QRD1114_PIN A0 // phototransistor


bool debug = true;
bool debugTime = false;
bool drive = true;

// driving var
float dutyCycle = 0;
int timeinms = 0;

// Timer
unsigned long timeZero, timePrev;
unsigned long timeAtTarget;

// Serila com var
char incomingMsg;
boolean newData = false;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// MPU6050 var
Adafruit_MPU6050 mpu_weight;
Adafruit_MPU6050 mpu_pred;

// Jump prediction variables
float y_avg[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int i = 0;
int maxOrMinSetI = -1;
int statusStart = 0;
float max_y_acc = -9999;
float min_y_acc = 9999;
String jumpStatus = "none";
int statusCounter = 0;
float predictJumpBeginThreshold = 0;
float powerJumpDutyCycle = -.5;
float powerJumpDriveDuration = 100;

/** Initiate VescUart class */
VescUart UART;

float current = 1.0; /** The current in amps */
float currentBrake = 20.0;

int firstDrive = 0;

// Weight position variables
float distance = 0;
float interval = 0.25;
String currentColor = "";
String moveDirection = "up";


int frames = 0;
float printInterval = 3000;
float lastPrint = 0;

String driveStatus = "none";

String controlMode = "arduino";
boolean powerJump = false;
String powerJumpPhase = "none";



void setup() {
  if (debug) Serial.begin(9600);

  while (!Serial); // Hold the code until serial monitor opens
  
  if (debug) Serial.println("JumpAR starting...");

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(19200);
 
  while (!Serial1) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);

//  UART.nunchuck.lowerButton = true;
//  UART.setNunchuckValues();

//  //if (debug) Serial.println("program starting...");

//  if (debug) Serial.print("debug: ");
//  if (debug) Serial.print(debug);
//  if (debug) Serial.print(" -- ");
//  if (debug) Serial.print(" debug time:/ ");
//  if (debug) Serial.print(debugTime);
//  if (debug) Serial.print(" -- ");
//  if (debug) Serial.print(" drive: ");
//  if (debug) Serial.print(drive);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SWITCH, INPUT);
  pinMode(QRD1114_PIN, INPUT);

  initMPU6050_pred();

//  initMPU6050_weight();
}

void loop() {
  // Predict user's jump status
    predictJumpStatus();

  // Perform powered jump if we are in powered jump mode and the user jumps
  if (controlMode == "arduino" && powerJump) {
      if (jumpStatus == "launching" && powerJumpPhase == "none") {
          startJump(powerJumpDutyCycle,powerJumpDriveDuration);
          powerJumpPhase = "up";
      }
      else if (jumpStatus == "falling" && powerJumpPhase == "up" && driveStatus == "initial")
      {
        driveStatus = "brake";
        if (debug) Serial.println("Apex reached, stopping drive early");
      }
      else if (jumpStatus == "none" && powerJumpPhase == "up") {
          if (powerJumpDutyCycle < 0)
            startJump(.05,4000);
          powerJumpPhase = "down";
      }
      else if (powerJumpPhase == "down" && driveStatus == "none")
      {
        powerJumpPhase = "none";
      }
  }
  

  // Get position of the weight.
  getPosition();
  /** Driving logic **/
  if (drive){
      recvWithStartEndMarkers();
      if (newData == true) {
          strcpy(tempChars, receivedChars);
              // this temporary copy is necessary to protect the original data
              //   because strtok() used in parseData() replaces the commas with \0
          String command = parseData();
          newData = false;
          if (command == "drive") startJump(dutyCycle, timeinms);
      }
    driveMotor();
  // Keep ESC awake
//  UART.setDuty(0);
  }
    
    
  if (millis() - lastPrint > printInterval){
    if (debug) Serial.print(frames);
    if (debug) Serial.print("Position: ");
    if (debug) Serial.println(distance);
    lastPrint = millis();
  }
  frames++;
}


void driveMotor(){
  if (driveStatus == "initial") {
    digitalWrite(LED_BUILTIN, HIGH); // visual marker
    if (timeAtTarget > millis() - timeZero) {
      // limit switch when driving down
      if (dutyCycle > 0){
        if (digitalRead(SWITCH) == 1) driveStatus = "brake"; 
      }
      // limit switch when driving up
      if (dutyCycle < 0){
        if (distance > 12.5) driveStatus = "brake"; 
      }
      if (driveStatus != "brake") {
        if (millis() - timePrev >= 900 || firstDrive == 0) { // send message every 900ms so it doesn't fill up ESC buffer
          if (drive) UART.setDuty(dutyCycle);
          timePrev = millis();
          firstDrive = 1;
        }
      }
    }
    else {
      driveStatus = "brake";
    }
  }
  if (driveStatus == "brake") {
      digitalWrite(LED_BUILTIN, LOW); // visual marker
      if (drive) {
        UART.setDuty(0);
        driveStatus = "none";
      }
  }
  if (driveStatus == "none") {
    if (frames % 25 == 0)
   UART.setDuty(0);
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

String parseData(){
    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the duty cycle
    if (strtokIndx[0] == 'p')
    { 
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJump = atoi(strtokIndx) == 1? true : false;     // convert this part to an integer
       if (debug) Serial.print("Set powerJump to ");
       if (debug) Serial.println(powerJump);
       return "";
    }
    else if (strtokIndx[0] == 'c')
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       controlMode = strtokIndx[0] == 'a' ?  "arduino" : "unity";     // convert this part to an integer
       if (debug) Serial.print("Set controlMode to ");
       if (debug) Serial.println(controlMode);
       return "";
    }
    else if (strtokIndx[0] == 't')
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       predictJumpBeginThreshold = atoi(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set prediction timing (m/s^2 above the minimum y acc) to ");
       if (debug) Serial.println(predictJumpBeginThreshold);
       return "";
    }
    else if (strtokIndx[0] == 'd')      // not used
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJumpDutyCycle = atof(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set power jump duty cycle to ");
       if (debug) Serial.println(powerJumpDutyCycle);
       return "";
    }
    else if (strtokIndx[0] == 's')
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJumpDutyCycle = atof(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set power jump duty cycle to ");
       if (debug) Serial.println(powerJumpDutyCycle);
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJumpDriveDuration = atof(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set power jump drive duration to ");
       if (debug) Serial.println(powerJumpDriveDuration);       
       return "";
    }
    else
    {
      dutyCycle = atof(strtokIndx);  // copy it to messageFromPC
      strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
      timeinms = atoi(strtokIndx);     // convert this part to an integer
      return "drive";
    }
    
}

void initMPU6050_pred(){
//  //if (debug) Serial.println("Adafruit MPU6050 init");

  // Try to initialize!
  if (!mpu_pred.begin(0x69)) {
    if (debug) Serial.println("Failed to find MPU6050 jump prediction chip");
    while (1) {
      delay(10);
    }
  }
//  //if (debug) Serial.println("MPU6050 Found!");

  mpu_pred.setAccelerometerRange(MPU6050_RANGE_8_G);
//  if (debug) Serial.print("Accelerometer range set to: ");
  switch (mpu_pred.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
//    //if (debug) Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
//    //if (debug) Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
//    //if (debug) Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
//    //if (debug) Serial.println("+-16G");
    break;
  }
  mpu_pred.setGyroRange(MPU6050_RANGE_500_DEG);
//  if (debug) Serial.print("Gyro range set to: ");
  switch (mpu_pred.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
//    //if (debug) Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
//    //if (debug) Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
//    //if (debug) Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
//    //if (debug) Serial.println("+- 2000 deg/s");
    break;
  }

  mpu_pred.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  if (debug) Serial.print("Filter bandwidth set to: ");
  switch (mpu_pred.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
//    //if (debug) Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
//    //if (debug) Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
//    //if (debug) Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
//    //if (debug) Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
//    //if (debug) Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
//    //if (debug) Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
//    //if (debug) Serial.println("5 Hz");
    break;
  }

//  //if (debug) Serial.println("");
  delay(100);
//  //if (debug) Serial.println("MPU initialized!");
}

void initMPU6050_weight(){
//  //if (debug) Serial.println("Adafruit MPU6050 init");

  // Try to initialize!
  if (!mpu_weight.begin(0x68)) {
    if (debug) Serial.println("Failed to find MPU6050 weight chip");
    while (1) {
      delay(10);
    }
  }
  //if (debug) Serial.println("MPU6050 Found!");

  mpu_weight.setAccelerometerRange(MPU6050_RANGE_8_G);
//  if (debug) Serial.print("Accelerometer range set to: ");
  switch (mpu_weight.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    //if (debug) Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    //if (debug) Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    //if (debug) Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    //if (debug) Serial.println("+-16G");
    break;
  }
  mpu_weight.setGyroRange(MPU6050_RANGE_500_DEG);
//  if (debug) Serial.print("Gyro range set to: ");
  switch (mpu_weight.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    //if (debug) Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    //if (debug) Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    //if (debug) Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    //if (debug) Serial.println("+- 2000 deg/s");
    break;
  }

  mpu_weight.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  if (debug) Serial.print("Filter bandwidth set to: ");
  switch (mpu_weight.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    //if (debug) Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    //if (debug) Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    //if (debug) Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    //if (debug) Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    //if (debug) Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    //if (debug) Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    //if (debug) Serial.println("5 Hz");
    break;
  }

  //if (debug) Serial.println("");
  delay(100);
  //if (debug) Serial.println("MPU initialized!");
}


void getMPU6050(unsigned long timeStart){
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu_weight.getEvent(&a, &g, &temp);

//  if (debug) Serial.print(millis() - timeStart);
//  if (debug) Serial.print(" , ");
//  if (debug) Serial.print(a.acceleration.x);
//  if (debug) Serial.print(" , ");
//  if (debug) Serial.print(a.acceleration.y);
//  if (debug) Serial.print(" , ");
//  if (debug) Serial.print(a.acceleration.z);
//  if (debug) Serial.print(" ; ");
//  if (debug) Serial.print(jumpStatus);
//  if (debug) Serial.print(" ");
//  //if (debug) Serial.println();
}

void printJumpStatus()
{  
   if (jumpStatus == "none")
   {
      if (debug) Serial.print(millis());
      if (debug) Serial.println(" 0 - none");
   }
   if (jumpStatus == "squatting")
   {
      if (debug) Serial.print(millis());
      if (debug) Serial.println(" 1 - squatting");
   }
   if (jumpStatus == "launching")
   {
      if (debug) Serial.print(millis());
      if (debug) Serial.println(" 2 - launching");
   }
   if (jumpStatus == "falling")
   {
      if (debug) Serial.print(millis());
      if (debug) Serial.println(" 3 - falling");
   }
}

void predictJumpStatus()
{
  sensors_event_t a, g, temp;
  mpu_pred.getEvent(&a, &g, &temp);
  if (debug) Serial.print("y - ");
  if (debug) Serial.println(a.acceleration.y);
  y_avg[i%20] = a.acceleration.y;
  i++;
  float y_mov_avg = 0;
  for (int j = 0; j < 20; j++)
  {
    y_mov_avg+=y_avg[j];
  }
  y_mov_avg/=20;
  
  if (max_y_acc < a.acceleration.y)
  {
    max_y_acc = a.acceleration.y;
    maxOrMinSetI = i;
  }

  if (max_y_acc < y_mov_avg)
  {
    max_y_acc = y_mov_avg;
    maxOrMinSetI = i;
  }


  if (min_y_acc > a.acceleration.y)
  {
    min_y_acc = a.acceleration.y;
    maxOrMinSetI = i;
  }

  if (min_y_acc > y_mov_avg)
  {
    min_y_acc = y_mov_avg;
    maxOrMinSetI = i;
  }

  if (jumpStatus == "none")
  {
        if (y_mov_avg > 12)
        {
            jumpStatus = "squatting";
            statusStart = millis();
            max_y_acc = -9999;
            printJumpStatus();
        }
  }
   else if (jumpStatus == "squatting")
   {
       if (y_mov_avg < max_y_acc - predictJumpBeginThreshold && i > 20 + maxOrMinSetI)
       {
            if (millis() - statusStart < 70)
            {
              if (debug) Serial.println("False jump (squatting time too short)");
              jumpStatus = "falling";
            }
            else
            {
                jumpStatus = "launching";
                min_y_acc = 9999; //Reset min_y_acc. We only care about the min_y_acc in the launching phase
                printJumpStatus();
                statusCounter = 0;
            }
       }
   }
    else if (jumpStatus == "launching")
    {
        statusCounter++;
        if (statusCounter > 800)
        {
            jumpStatus = "falling";
            printJumpStatus();
            statusCounter = 0;
        }
       if (y_mov_avg > min_y_acc + 1 && min_y_acc < 0 && i > 20 + maxOrMinSetI) 
       {
            jumpStatus = "falling";
            printJumpStatus();
            statusCounter = 0;
       }
    }
    else if (jumpStatus == "falling")
    {
        statusCounter++;
        if (statusCounter > 800)
        {
            jumpStatus = "none";
            printJumpStatus();
            statusCounter = 0;
            max_y_acc = -9999;
        }
    }
}

void getPosition()
{
  int proximity = digitalRead(QRD1114_PIN);
    if (proximity)
    {
      if (currentColor == "black")
      {
        distance+=interval * (moveDirection == "up" ? 1 : -1);
      }
      currentColor = "white";              
    }
    else
    {
      if (currentColor == "white")
      {
        distance+=interval * (moveDirection == "up" ? 1 : -1);
      }
      currentColor = "black";
    }
    if (digitalRead(SWITCH))
    {
      distance = 0;
      moveDirection = "up";
    }
}

void startJump(float newDutyCycle, int newTimeInMS)
{
  dutyCycle = newDutyCycle;
  timeinms = newTimeInMS;

  // Init params for timer
  firstDrive = 0;
  timeZero = millis();
  timePrev = timeZero;
  timeAtTarget = timeinms;// + timeZero;
  driveStatus = "initial";
  if (debug) Serial.print("Duty cycle: ");
  if (debug) Serial.print(dutyCycle);
  if (debug) Serial.print(" timing: ");
  if (debug) Serial.println(timeinms);
  if (dutyCycle < 0) moveDirection = "up";
  else moveDirection = "down";
}
