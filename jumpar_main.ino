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
#include <math.h>

#define SWITCH_BOT 10 // limit switch bottom
#define SWITCH_TOP 11 // limit switch top
#define RELAY 12      // relay to control subwoofer
#define QRD1114_PIN A0 // phototransistor


bool debug = true;
bool debugJumpAccel = false;
bool debugSW = false;
bool debugTime = false;
bool drive = true;
bool sendCalib = true;
bool onlyPrediction = false;
bool motorLED = false; // visual marker for driving debugging
bool predLED = true;   // visual marker for prediction debugging

// driving var
float dutyCycle = 0;
int timeinms = 0;
int predStartDrive = 3;
int predStopDrive = 4;

// Timer
unsigned long timeZero, timePrev;
unsigned long timeAtTarget;

// Serial com var
char incomingMsg;
boolean newData = false;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// Subwoofer
int subwooferStatus = 0; // 0 is off, 1 is on

// MPU6050 var
Adafruit_MPU6050 mpu_weight;
Adafruit_MPU6050 mpu_pred;

// Jump prediction variables
float y_avg[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int i = 0; //increments every frame
int i2 = 0; //increments every jump
int iMarker = -1;
int statusStart = 0;
float max_y_acc = -9999;
float min_y_acc = 9999;
float velocity = 0;
float maxVelocity = -999;
float minVelocity = 999;
float apexVelocity[] = {75,75,75,75,75};
float apexAverage = 75;
float status7Velocity[] = {-5,-5,-5,-5,-5};
float status7Average = -5;
int jumpStatus = 1;
int jumpTimeMode = 0;
int statusCounter = 0;
float powerJumpDutyCycle = -.5;
float powerJumpDriveDuration = 100;
float YZ = 0;
float lastPredict = 0;

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
  /** Setup serial port for computer com **/
  Serial.begin(9600);

//  while (!Serial); // Hold the code until serial monitor opens
  
  Serial.println("JumpAR program begins...");

  if (!onlyPrediction){
    /** Setup UART port for VESC (Serial1 on Atmega32u4) */
    Serial1.begin(115200);
   
    //  while (!Serial1); // Hold the code until it can connect to VESC
  
    /** Define which ports to use as UART */
    UART.setSerialPort(&Serial1);
  
    Serial.println("UART initialization successful!");
  
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
  
    pinMode(SWITCH_BOT, INPUT);
    pinMode(SWITCH_TOP, INPUT);
    pinMode(QRD1114_PIN, INPUT);
    pinMode(RELAY, OUTPUT);
  
    digitalWrite(RELAY, LOW); // default turn off subwoofer
  }

  initMPU6050_pred();
  Serial.println("IMU initialization successful!");

//  initMPU6050_weight();

//  delay(5000);
}

void loop() {
  // Predict user's jump status
  predictJumpStatus();
  

  if (!onlyPrediction){
    // Perform powered jump if we are in powered jump mode and the user jumps
    if (controlMode == "arduino") {
        if (jumpStatus == predStartDrive && powerJumpPhase == "none" && powerJump) { // condition to start drive
            startJump(powerJumpDutyCycle,powerJumpDriveDuration);
            powerJumpPhase = "up";
        }
        else if (jumpStatus == predStopDrive && powerJumpPhase == "up" && driveStatus == "initial" && powerJump) // condition to stop drive
        {
          driveStatus = "brake";
          if (debug) Serial.println("Apex reached, stopping drive early");
        }
        else if (jumpStatus == 1 && powerJumpPhase == "up") { // condition to reset weight
            if (powerJumpDutyCycle < 0)
              startJump(.05,4000);
            else if (powerJumpDutyCycle > 0)
              startJump(-.05,4000);
            powerJumpPhase = "down";
        }
        else if (powerJumpPhase == "down" && driveStatus == "none")
        {
          powerJumpPhase = "none";
        }  
    }
  
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

    // Subwoofer logic
    if (subwooferStatus == 1){
      digitalWrite(RELAY, LOW);
    }
    else{
      digitalWrite(RELAY, HIGH);
    }
  
    // auto driving - debugging reliability
    //  startJump(-0.1, 100);
    //  driveMotor();
    
    //  delay(2000);
    //
    //  startJump(0.1, 100);
    //  driveMotor();
    //
    //  delay(2000);
  
    if (debugSW){
      Serial.print("SW_B = ");
      Serial.print(digitalRead(SWITCH_BOT));
      Serial.print(" -- ");
      Serial.print("SW_T = ");
      Serial.println(digitalRead(SWITCH_TOP));
      
    }
  }
  
    
  if (millis() - lastPrint > printInterval){
//    if (debug) Serial.print(frames);
//    if (debug) Serial.print("YZ acc magnitude:");
//    if (debug) Serial.print(YZ);
//    if (debug) Serial.print(", Velocity: ");
//    if (debug) Serial.print(velocity);
//    if (debug) Serial.print(", Apex Vel Avg: ");
//    if (debug) Serial.print(apexAverage); 
//    if (debug) Serial.print(", Status 7 Vel Avg: ");
//    if (debug) Serial.println(status7Average);
    if (sendCalib) Serial.print("Calibration: ");
    if (sendCalib) Serial.println(apexAverage); 
    lastPrint = millis();
  }
  frames++;
}


void driveMotor(){
  if (driveStatus == "initial") {
    if (motorLED) digitalWrite(LED_BUILTIN, HIGH); // visual marker
    if (timeAtTarget > millis() - timeZero) {
      // limit switch when driving down
      if (dutyCycle > 0){
        if (digitalRead(SWITCH_BOT) == 1) driveStatus = "brake"; 
      }
      // limit switch when driving up
      if (dutyCycle < 0){
        if (digitalRead(SWITCH_TOP) == 1) driveStatus = "brake"; 
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
      if (motorLED) digitalWrite(LED_BUILTIN, LOW); // visual marker
      if (drive) {
        UART.setDuty(0);
        driveStatus = "none";
      }
  }
  if (driveStatus == "none") {
    if (frames % 100 == 0)
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

    strtokIndx = strtok(tempChars,","); // arming the device: <p,0> = disarm, <p,1> = arm
    if (strtokIndx[0] == 'p')
    { 
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJump = atoi(strtokIndx) == 1? true : false;     // convert this part to an integer
       if (debug) Serial.print("Set powerJump to ");
       if (debug) Serial.println(powerJump);
       return "";
    }
    else if (strtokIndx[0] == 'c') // setting the way the backpack is controlled, arduino or unity
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       controlMode = strtokIndx[0] == 'a' ?  "arduino" : "unity";     // convert this part to an integer
       if (debug) Serial.print("Set controlMode to ");
       if (debug) Serial.println(controlMode);
       return "";
    }
    else if (strtokIndx[0] == 't') // setting different timing modes
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       predStartDrive = atoi(strtokIndx);     // convert this part to an integer
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       predStopDrive = atoi(strtokIndx);
       if (debug) Serial.print("Start driving at mode ");
       if (debug) Serial.print(predStartDrive);
       if (debug) Serial.print(" and stop driving at mode ");
       if (debug) Serial.println(predStopDrive);
       return "";
    }
    else if (strtokIndx[0] == 'd') // not used
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       powerJumpDutyCycle = atof(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set power jump duty cycle to ");
       if (debug) Serial.println(powerJumpDutyCycle);
       return "";
    }
    else if (strtokIndx[0] == 's') // setting the jump DC and timing
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
    else if (strtokIndx[0] == 'w') // turning on/off the subwoofer
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       subwooferStatus = atof(strtokIndx);     // convert this part to an integer
       if (debug) Serial.print("Set subwoofer to ");
       if (debug) Serial.println(subwooferStatus);  
       return "";
    }
    else if (strtokIndx[0] == 'v') // set velocity of the apex
    {
       strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
       apexAverage = atof(strtokIndx);     // convert this part to an integer
       apexVelocity[0] = atof(strtokIndx);
       apexVelocity[1] = atof(strtokIndx);
       apexVelocity[2] = atof(strtokIndx);
       apexVelocity[3] = atof(strtokIndx);
       apexVelocity[4] = atof(strtokIndx);
       if (debug) Serial.print("Set apex velocity to ");
       if (debug) Serial.println(apexAverage);  
       return "";
    }
    else
    { // manual mode
      dutyCycle = atof(strtokIndx);  // copy it to messageFromPC
      strtokIndx = strtok(NULL, ","); // get the 2nd part - the interation 
      timeinms = atoi(strtokIndx);     // convert this part to an integer
      return "drive";
    }
    
}

void initMPU6050_pred(){
//  //if (debug) Serial.println("Adafruit MPU6050 init");

  // Try to initialize!
  if (!mpu_pred.begin(0x68)) { //69 if ADDR pulled high
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
   if (debug) Serial.print("Jump Status = ");
   if (debug) Serial.println(jumpStatus);
   statusStart = millis();
}

void predictJumpStatus()
{
  sensors_event_t a, g, temp;
  mpu_pred.getEvent(&a, &g, &temp);

//  float angle = asin(min(9.8,abs(a.acceleration.z))/9.8);
//  float trueYAcc = a.acceleration.y / cos(angle);
  YZ = sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  y_avg[i%20] = YZ;
  i++;
  float y_mov_avg = 0;
  for (int j = 0; j < 20; j++)
  {
    y_mov_avg+=y_avg[j];
  }
  y_mov_avg/=20;

  velocity += YZ - 9.8;
  if (abs(y_avg[i%20] - y_avg[(i-1)%20]) < .05 && abs(y_avg[i%20] - 9.8) < 2)
    velocity = velocity * 0.95;

  if (debugJumpAccel) Serial.print("y - ");
  if (debugJumpAccel) Serial.print(a.acceleration.y);
  if (debugJumpAccel) Serial.print(" yz - ");
  if (debugJumpAccel) Serial.print(YZ);
  if (debugJumpAccel) Serial.print(" - velocity ");
  if (debugJumpAccel) Serial.println(velocity);
  
  if (velocity > maxVelocity)
  {
    maxVelocity = velocity;
    iMarker = i;
  }
  if (velocity < minVelocity)
  {
    minVelocity = velocity;
    iMarker = i;
  }

  if (jumpStatus == 1)
  {
        if (velocity < -100 && i > 3 + iMarker)
        {
            jumpStatus = 2;
            printJumpStatus();
        }
  }
   else if (jumpStatus == 2)
   {
       if (velocity > 0)
       {
           jumpStatus = 3;
           if (predLED) digitalWrite(LED_BUILTIN, HIGH);
           maxVelocity = -9999;
           printJumpStatus();            
       }
   }
    else if (jumpStatus == 3)
    {
       if (velocity < maxVelocity and i > 3 + iMarker) 
       {
            jumpStatus = 4;
            if (predLED) digitalWrite(LED_BUILTIN, LOW);
            printJumpStatus();
       }
    }
    else if (jumpStatus == 4)
    {
       if (velocity < maxVelocity / 2)
        {
            jumpStatus = 5;
            if (predLED) digitalWrite(LED_BUILTIN, HIGH);
            printJumpStatus();
        }
    }
    else if (jumpStatus == 5)
    {
      if (velocity < apexAverage)
      {
        jumpStatus = 6;
        if (predLED) digitalWrite(LED_BUILTIN, LOW);
        minVelocity = 9999;
        printJumpStatus();
      }
    }
    else if (jumpStatus == 6)
    {
      if (velocity < status7Average)
      {
        jumpStatus = 7;
        if (predLED) digitalWrite(LED_BUILTIN, HIGH);
        printJumpStatus();
      }
    }
    else if (jumpStatus == 7)
    {
      if (velocity > minVelocity and i > 3 + iMarker)
      {
        jumpStatus = 8;
        if (predLED) digitalWrite(LED_BUILTIN, LOW);
        printJumpStatus();
        apexVelocity[i2%5] = maxVelocity - (maxVelocity - minVelocity) / 2;
        apexAverage = (apexVelocity[0] + apexVelocity[1] + apexVelocity[2] + apexVelocity[3] + apexVelocity[4])/5;
        status7Velocity[i2%5] = apexVelocity[i2%5] - (apexVelocity[i2%5] - minVelocity) / 2;
        status7Average = (status7Velocity[0] + status7Velocity[1] + status7Velocity[2] + status7Velocity[3] + status7Velocity[4])/5;
        i2++;

      }
    }
    else if (jumpStatus == 8)
    {
      if (velocity > minVelocity / 2)
      {
        jumpStatus = 9;
        if (predLED) digitalWrite(LED_BUILTIN, HIGH);
        printJumpStatus();
      }
    }
    else if (jumpStatus == 9)
    {
//      if (velocity > -100)
//      {
//        jumpStatus = 1;
//        minVelocity = 9999;
//        printJumpStatus();
//      }
    }
    if (jumpStatus != 1 && millis() - statusStart > 1500)
    {
      minVelocity = 9999;
      maxVelocity = -9999;
      jumpStatus = 1; 
      if (predLED) digitalWrite(LED_BUILTIN, LOW);
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
