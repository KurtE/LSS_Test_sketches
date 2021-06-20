// LSS Setup & Variables
#include <Wire.h>
#include "SparkFun_Qwiic_Keypad_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_keypad
KEYPAD keypad1; //Create instance of this object


#include <LSS.h>
#define LSS_BAUD  (921600)

LSS LSS_1 = LSS(0); 

int Timing = 250;                        // Time in ms for the motion (T Parameter) 
int delais_frame = 1000;                  // Time between each Frame
int em0_rate = 25;                        // Rate at wich the positions are sent (20ms)

void setup() {

  Serial.begin(115200);
  
  if (keypad1.begin() == false)   // Note, using begin() like this will use default I2C address, 0x4B. 
                  // You can pass begin() a different address like so: keypad1.begin(Wire, 0x4A).
  {
    Serial.println("Keypad does not appear to be connected. Please check wiring. Freezing...");
    while (1);
  }
  Serial.print("Initialized. Firmware Version: ");
  Serial.println(keypad1.getVersion());
  
// Initialize the LSS bus
  LSS::initBus(Serial1, LSS_BAUD);
  Serial.println("Initial Reset: ");
  LSS_1.reset();
  delay(2000);
  LSS_1.setServoID(6);
  // LSS Settings
  Serial.println("Set AngularStiffness to -4: ");
  LSS_1.setAngularStiffness(-4);
  Serial.println("");
  delay(250);
  Serial.print("Set Holding Stiffness to 4: ");
  LSS_1.setAngularHoldingStiffness(4);
  Serial.println("");
  delay(250);
  Serial.println("Set Motion Control: ");
  LSS_1.setMotionControlEnabled(0);
  delay(250);
  LSS_1.setMaxSpeed(1800, LSS_SetSession);

  
  Serial.println("Do not Set FPC: ");
  //Serial1.println("#6FPC5\r");
  delay(250);
  #ifdef LSS_SupportsSettingTimeouts
  LSS::setReadTimeouts(20, 5); // define we will wait for 20ms for response to start and 5ms for in message characters
  #endif
  
}

void sequence(){
  // Frame 1:
  Serial.printf("\nFRAME 1\n--------------\n");
  Spoofing(-450,Timing);
  delay(delais_frame*3);
  Serial.printf("\nFRAME 2\n--------------\n");
  // Frame 2:
  Spoofing(450,Timing);
  delay(delais_frame);
  }

void Spoofing(int32_t LSS_S_1,int32_t p2_timing){
  int pos_num = p2_timing/em0_rate;
  int32_t temp[pos_num];
  uint32_t stime;
  
  int32_t   LSS_1_qd = query_LSS_1();
  int32_t   LSS_1_delta = LSS_S_1 - LSS_1_qd;
  int32_t     LSS_1_min_pos = LSS_1_delta/pos_num;
  Serial.printf("goal: %d, time: %d, pos_num: %d\n", LSS_S_1, p2_timing, pos_num);
  Serial.printf("Current pos: (qd: %d), dPos(%d), tStep(%d)\n", LSS_1_qd, LSS_1_delta, LSS_1_min_pos); 

  temp[0] = LSS_1.getPosition();
  stime = millis();
  for (int32_t i = 1; i < pos_num+1; i++){
    Serial.printf("i: %d, move: %d\n", i, (LSS_1_min_pos * i) + LSS_1_qd);
    LSS_1.move((LSS_1_min_pos * i) + LSS_1_qd);
    delay(em0_rate);
    checkStatus();
    temp[i] = query_LSS_1();
  } 
    LSS_1.move(LSS_S_1);
    delay(2);
    Serial.printf("Final Move: %d\n", LSS_S_1);
    uint32_t dt = millis() - stime;
    
    Serial.println("Time(ms)/ Pos(deg)");
    Serial.printf("%d / ", dt);
    for(int i = 0; i < pos_num; i++) {
      Serial.printf("%d, ", temp[i]);
    }
    Serial.print(LSS_1.getPosition()); 
    Serial.println();
    
}

int32_t query_LSS_1()
{
  int32_t posLSS = 0;
  int32_t lastPosLSS = -1;
  char readBuffer[100];
  lastPosLSS = posLSS;
  Serial1.write("#6QD\r");
  while(Serial1.available() == 0)
  {
  }
  size_t readLength = 0;
  readLength = Serial1.readBytesUntil('D', readBuffer, 100);  // Read until after the command (QD), indicating the start of the returned value (position in 1/10 deg)
  readLength = Serial1.readBytesUntil('\r', readBuffer, 100); // Read until the carriage return (\r), indicating the end of the reply
  readBuffer[readLength]=0;
  if(readLength > 0)
  {
    if(LSS_1.charToInt(readBuffer, &posLSS))
    {
      Serial.printf("Returned Postition: %d\n", posLSS);
      return posLSS;
    }
  } 
}

void loop() {
  keypad1.updateFIFO();  // necessary for keypad to pull button from stack to readable register
  char button = keypad1.getButton();
  
      // only toggle the LED if the new button state is HIGH
       if (button == '#') {
        //Start the sequence
        Serial.println("Sequence Start");
        sequence(); 
      }                  
}

void checkStatus()
{
  int8_t status1 = -1;
  uint32_t statusTime = 0;
  while (status1 != 6) {
    if (status1 != 6) status1 = LSS_1.getStatus();
    //delay(2);
    statusTime += 1;
  }
  Serial.printf("Status Loop CNT: %d: %d\n", statusTime, status1);
}
