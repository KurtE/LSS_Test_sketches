static const uint32_t from_speed = 115200;
static const uint32_t to_speed = 500000;
void setup()
{
  while (!Serial && millis() < 5000) ;
  Serial.printf("Change all LSS Servos from %u to %u baud\n", from_speed, to_speed);
}
void loop() {
  Serial1.begin(from_speed);
  Serial.printf("Scan servos at %u\n", from_speed);
  bool Servos_found = ScanServos();
  if (Servos_found) {
    Serial.println("Press any key to continue");
    while (Serial.read() == -1) ;
    while (Serial.read() != -1) ;
    delay(250);
    Serial.println("Try set baud cammand");
    Serial1.printf("#254CB%u\r", to_speed);
    delay(250);
    Serial.println("Reset");
    Serial1.print("#254RESET\r");
    delay(3000);
    Serial1.end();
    Serial1.begin(to_speed);
    delay(250);
    Serial.printf("Scan servos at %u\n", to_speed);
    ScanServos();
  }
  Serial1.end();
  Serial.println("Press any key to try again");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1) ;
}

bool ScanServos() {
  bool found_servo = false;
  for (uint8_t servo_id = 0; servo_id < 254; servo_id++) {
    Serial1.printf("#%uQD\r", servo_id);
    Serial1.flush();
    delay(25);
    if (Serial1.available()) {
      int ch;
      while ((ch = Serial1.read()) != -1) {
        Serial.write(ch);
      }
      found_servo = true;
      Serial.println();
    }
  }
  return found_servo;
}
