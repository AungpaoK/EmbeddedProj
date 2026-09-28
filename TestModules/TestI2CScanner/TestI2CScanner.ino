#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println(F("I2C scanner ready"));
}

void loop() {
  byte found = 0;
  Serial.println(F("Scanning I2C addresses..."));

  for (byte address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("Found 0x"));
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    }
  }

  if (found == 0) {
    Serial.println(F("No I2C devices found"));
  }
  Serial.println(F("Scan complete; rescanning in 3 seconds"));
  delay(3000);
}
