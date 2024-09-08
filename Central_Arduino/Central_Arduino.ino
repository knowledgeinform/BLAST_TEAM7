#include <SPI.h>
#include <RH_RF95.h>

// Define the pins for the RFM95W
#define RFM95_CS 4
#define RFM95_RST 5
#define RFM95_INT 3

// Define frequency for RFM95W
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Define structures for USV data
struct USVStatus {
  int gridX;
  int gridY;
  float latitude;
  float longitude;
  int signalStrength;
  char id[3];
};

// Array of USV IDs
const uint8_t USV_IDS[] = {1, 2, 3}; // IDs for USV1, USV2, USV3
const uint8_t CENTRAL_ID = 99; // Central unit's ID

// Maximum number of retries for each USV
const int MAX_RETRIES = 3;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);

  // Set up the reset pin
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Manual reset for the RFM95 module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the LoRa radio
  if (!rf95.init()) {
    Serial.println("RFM95 LoRa init failed");
    while (1);
  }
  Serial.println("RFM95 LoRa init OK!");

  // Set frequency for RFM95
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("RFM95 setFrequency failed");
    while (1);
  }
  Serial.print("RFM95 Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Set transmission power
  rf95.setTxPower(23, false); // Using PA_BOOST

  Serial.println("Central Arduino Listening for Data...");
}

void requestDataFromUSV(uint8_t usvId) {
  rf95.setHeaderFrom(CENTRAL_ID); // Central unit ID
  rf95.setHeaderTo(usvId);        // Request to specific USV ID

  // Send a simple request message
  const char* message = "REQ";
  rf95.send((uint8_t*)message, strlen(message));
  rf95.waitPacketSent();
  Serial.print(F("Request sent to USV"));
  Serial.println(usvId);
}

bool waitForResponse(uint8_t usvId, USVStatus &usvStatus, float &estLat, float &estLon) {
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) { // 1-second timeout
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (rf95.recv(buf, &len)) {
        uint8_t from = rf95.headerFrom();
        uint8_t to = rf95.headerTo();

        if (from == usvId && to == CENTRAL_ID) {
          if (len == sizeof(USVStatus)) {
            memcpy(&usvStatus, buf, sizeof(usvStatus));
            return true;
          } else if (len >= 18 && strncmp((char *)buf, "ESTIMATED", 9) == 0) {
            char *token = strtok((char *)buf, ",");
            token = strtok(NULL, ",");
            if (token) estLat = atof(token);
            token = strtok(NULL, ",");
            if (token) estLon = atof(token);
            Serial.print("ESTIMATED,");
            Serial.print(estLat, 6);
            Serial.print(",");
            Serial.println(estLon, 6);
            return true;
          }
        }
      }
    }
  }
  return false;
}

void loop() {
  USVStatus usvStatus;
  float estimatedLat = 0;
  float estimatedLon = 0;

  // Sequentially request data from each USV
  for (uint8_t i = 0; i < sizeof(USV_IDS); i++) {
    int retries = 0;
    bool responseReceived = false;

    while (retries < MAX_RETRIES && !responseReceived) {
      requestDataFromUSV(USV_IDS[i]);

      if (waitForResponse(USV_IDS[i], usvStatus, estimatedLat, estimatedLon)) {
        responseReceived = true;

        if (estimatedLat != 0 && estimatedLon != 0) {
          // If an estimated position is received, it will be printed already
          continue;
        }

        Serial.print("DATA,");
        Serial.print(usvStatus.id);
        Serial.print(",");
        Serial.print(usvStatus.latitude, 6);
        Serial.print(",");
        Serial.print(usvStatus.longitude, 6);
        Serial.print(",");
        Serial.println(usvStatus.signalStrength);
      } else {
        retries++;
        Serial.print("Retrying... (");
        Serial.print(retries);
        Serial.println(")");
      }
    }

    if (!responseReceived) {
      Serial.print("Failed to get response from USV");
      Serial.println(USV_IDS[i]);
    }
  }
}
