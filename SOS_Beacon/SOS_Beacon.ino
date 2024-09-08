#include <SPI.h>
#include <RH_RF95.h>

// Define the pins for RFM96W
#define RFM96_CS 10
#define RFM96_RST 9
#define RFM96_INT 2

// Singleton instance of the radio driver
RH_RF95 rf96(RFM96_CS, RFM96_INT);

// Define the frequency for the RFM96W modulea
#define RF96_FREQ 433.0

void setup() {
  // Set up the reset pin
  pinMode(RFM96_RST, OUTPUT);
  digitalWrite(RFM96_RST, HIGH);

  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa SOS Beacon");

  // Perform manual reset
  digitalWrite(RFM96_RST, LOW);
  delay(10);
  digitalWrite(RFM96_RST, HIGH);
  delay(10);

  // Initialize the radio
  if (!rf96.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Set the radio frequency
  if (!rf96.setFrequency(RF96_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF96_FREQ);

  // Set the transmission power
  rf96.setTxPower(23, false);  // Using PA_BOOST
}

void loop() {
  const char *msg = "SOS";
  rf96.send((uint8_t *)msg, strlen(msg));
  rf96.waitPacketSent();
  Serial.println("SOS message sent");
  delay(1000);  // Send SOS every second
}
