#include <LSS.h>
// ID set to default LSS ID = 0
#define LSS_ID		(1)
#define LSS_BAUD	(250000)
#define LSS_SERIAL  (Serial1) // ex: Teensy


LSS myLSS = LSS(LSS_ID);

void PrintServoStuff() {
  Serial.print("Pos:");
  Serial.print(myLSS.getPosition(), DEC);
  Serial.print(" Comm Status:");
  Serial.print(myLSS.getLastCommStatus(), DEC);
  Serial.print("\tO:");
  Serial.print(myLSS.getOriginOffset(), DEC);
  Serial.print(":");
  Serial.print(myLSS.getOriginOffset(LSS_QueryConfig), DEC);
  Serial.print("\tG:");
  Serial.print(myLSS.getGyre(), DEC);
  Serial.print(":");
  Serial.print(myLSS.getGyre(LSS_QueryConfig), DEC);
  Serial.print("\tEMC:");
  Serial.print(myLSS.getIsMotionControlEnabled(), DEC);
  //Serial.print(":");
  //Serial.print(myLSS.getIsMotionControlEnabled(LSS_QueryConfig), DEC);
  Serial.print("\tFPC:");
  Serial.print(myLSS.getFilterPositionCount(), DEC);
  Serial.print(":");
  Serial.print(myLSS.getFilterPositionCount(LSS_QueryConfig), DEC);
  Serial.print("\tAS:");
  Serial.print(myLSS.getAngularStiffness(), DEC);
  Serial.print(":");
  Serial.print(myLSS.getAngularStiffness(LSS_QueryConfig), DEC);
  Serial.print("\tAH:");
  Serial.print(myLSS.getAngularHoldingStiffness(), DEC);
  Serial.print(":");
  Serial.print(myLSS.getAngularHoldingStiffness(LSS_QueryConfig), DEC);
  Serial.print("\tAR:");
  Serial.print(myLSS.getAngularRange(), DEC);
  Serial.print(":");
  Serial.println(myLSS.getAngularRange(LSS_QueryConfig), DEC);

}
void userwait() {
  Serial.println("Hit any key to continue");
  while (Serial.read() == -1);
  while (Serial.read() != -1);
}
void setup()
{
  while (!Serial && millis() < 5000) ;

  // Initialize the LSS bus
  LSS::initBus(LSS_SERIAL, LSS_BAUD);

  // Initialize LSS to position 0.0 °
  myLSS.setServoID(LSS_ID);
  myLSS.reset();
  delay(1500);
  myLSS.hold();
  myLSS.moveT(0,500);
  myLSS.setOriginOffset(300);
  myLSS.setGyre(LSS_GyreCounterClockwise);

  delay(500);
  PrintServoStuff();
  
  myLSS.moveT(500, 500);
  delay(1000);
  userwait();
  Serial.println("SetGyre to same setting");
  myLSS.setGyre(myLSS.getGyre());
  myLSS.setOriginOffset(myLSS.getOriginOffset());
  myLSS.moveT(0, 500);
  delay(500);
  PrintServoStuff();
  myLSS.moveT(500, 500);
  userwait();

  Serial.println("Set EMC=0");
  myLSS.setMotionControlEnabled(0);
  myLSS.setAngularHoldingStiffness(4, LSS_SetSession);
  myLSS.setAngularStiffness(-4, LSS_SetSession);
  myLSS.setFilterPositionCount(3, LSS_SetSession);
  PrintServoStuff();
  myLSS.move(0);
  myLSS.move(0);
  myLSS.move(0);

  delay(500);
  for (int pos = 0; pos <= 500; pos++) {  
    delay(1);
    myLSS.move(pos);
    delay(1);
    myLSS.move(pos);
  }
  myLSS.move(500);
  myLSS.move(500);
  myLSS.move(500);
  myLSS.move(500);


}

void loop()
{
}
