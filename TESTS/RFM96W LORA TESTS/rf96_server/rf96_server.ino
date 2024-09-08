#include <SPI.h>
#include <RH_RF95.h>

#define RFM96_CS 10
#define RFM96_RST 9
#define RFM96_INT 2

// Change frequency to 433.0 MHz
#define RF96_FREQ 433.0

// Singleton instance of the radio driver
RH_RF95 rf96(RFM96_CS, RFM96_INT);

void setup() 
{
  pinMode(RFM96_RST, OUTPUT);
  digitalWrite(RFM96_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM96_RST, LOW);
  delay(10);
  digitalWrite(RFM96_RST, HIGH);
  delay(10);

  while (!rf96.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  // Set frequency to 433.0 MHz
  if (!rf96.setFrequency(RF96_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF96_FREQ);

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf96.setTxPower(23, false);
}

void loop()
{
  if (rf96.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf96.recv(buf, &len))
    {
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf96.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf96.send(data, sizeof(data));
      rf96.waitPacketSent();
      Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
