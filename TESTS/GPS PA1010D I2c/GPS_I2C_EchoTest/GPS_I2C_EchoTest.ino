// Test code for Adafruit GPS That Support Using I2C
//
// This code shows how to test a passthru between USB and I2C
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);

void setup() {
  // wait for hardware serial to appear
  while (!Serial);

  // make this baud rate fast enough so we aren't waiting on it
  Serial.begin(115200);

  Serial.println("Adafruit GPS library basic I2C test!");

  // Initialize the GPS
  if (!GPS.begin(0x10)) { // The I2C address to use is 0x10
    Serial.println("Failed to initialize GPS! Please check wiring.");
    while (1); // Loop forever, halting the program
  }

  Serial.println("GPS initialized successfully!");

  // Request firmware version and configuration
  GPS.sendCommand(PMTK_Q_RELEASE);
  delay(1000);

  // Check if the GPS is responding
  if (GPS.available()) {
    Serial.println("GPS is responding.");
  } else {
    Serial.println("GPS is not responding. Check connections and power.");
    while (1); // Loop forever, halting the program
  }
}

void loop() {
  // Pass through data from Serial to GPS
  if (Serial.available()) {
    char c = Serial.read();
    GPS.write(c);
  }

  // Pass through data from GPS to Serial
  if (GPS.available()) {
    char c = GPS.read();
    Serial.write(c);
  }
}
