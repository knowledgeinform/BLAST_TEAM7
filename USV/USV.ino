#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <math.h>

// Define pins for RFM95W and RFM96W
#define RFM95_CS 4
#define RFM95_RST 5
#define RFM95_INT 3

#define RFM96_CS 10
#define RFM96_RST 9
#define RFM96_INT 2


// Define frequencies for RFM95W and RFM96W
#define RF95_FREQ 915.0  // Frequency for communication with other USVs
#define RF96_FREQ 433.0  // Frequency for receiving SOS signals

// Constants for signal processing
#define SPEED_OF_LIGHT 299792458  // Speed of light in m/s

// Singleton instances of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);  // RFM95W for communication
RH_RF95 rf96(RFM96_CS, RFM96_INT);  // RFM96W for SOS signal detection
Adafruit_GPS GPS(&Wire);            // Using I2C for GPS

// Structure to hold USV status
struct USVStatus {
  int8_t gridX;
  int8_t gridY;
  float latitude;
  float longitude;
  int8_t signalStrength;
  char id[3];
};

USVStatus myStatus;
USVStatus neighborStatuses[3]; // Reduced size to save memory
int neighborCount = 0;

// Define grid position for this USV
int8_t gridX, gridY;

// State management for USV roles
bool isCentralUSV = false;

// Function to set the grid position
void setGridPosition(int8_t x, int8_t y) {
  gridX = x;
  gridY = y;
}

// Function to update the USV's status
void updateStatus() {
  myStatus.gridX = gridX;
  myStatus.gridY = gridY;
  myStatus.latitude = GPS.latitudeDegrees;
  myStatus.longitude = GPS.longitudeDegrees;
  myStatus.signalStrength = (int8_t)rf96.lastRssi();
  strcpy(myStatus.id, "U3"); // Update with this USV's ID
}

// Function to broadcast the USV's status to other USVs
void broadcastStatus() {
  updateStatus();
  rf95.send((uint8_t *)&myStatus, sizeof(myStatus));
  rf95.waitPacketSent();
  Serial.println(F("Status broadcasted"));
} 

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);

  // Example of setting grid position
  setGridPosition(0, 0); // Example position; adjust accordingly

  // Set up the reset pins
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(RFM96_RST, OUTPUT);
  digitalWrite(RFM96_RST, HIGH);

  // Manual reset for the RFM95 module
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Manual reset for the RFM96 module
  digitalWrite(RFM96_RST, LOW);
  delay(10);
  digitalWrite(RFM96_RST, HIGH);
  delay(10);

  // Initialize the RFM95 radio
  if (!rf95.init()) {
    Serial.println(F("RFM95 LoRa init failed"));
    while (1);
  }
  Serial.println(F("RFM95 LoRa init OK!"));

  // Set frequency for RFM95
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("RFM95 setFrequency failed"));
    while (1);
  }
  Serial.print(F("RFM95 Set Freq to: "));
  Serial.println(RF95_FREQ);

  // Set transmission power for RFM95
  rf95.setTxPower(23, false); // Using PA_BOOST

  // Initialize the RFM96 radio
  if (!rf96.init()) {
    Serial.println(F("RFM96 LoRa init failed"));
    while (1);
  }
  Serial.println(F("RFM96 LoRa init OK!"));

  // Set frequency for RFM96
  if (!rf96.setFrequency(RF96_FREQ)) {
    Serial.println(F("RFM96 setFrequency failed"));
    while (1);
  }
  Serial.print(F("RFM96 Set Freq to: "));
  Serial.println(RF96_FREQ);

  // Set transmission power for RFM96
  rf96.setTxPower(23, false); // Using PA_BOOST

  // Initialize GPS module
  GPS.begin(0x10);  // I2C address for GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);
}

void loop() {
  // Read GPS data
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  // Check for incoming SOS signals on RFM96
  if (rf96.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf96.recv(buf, &len)) {
      if (strcmp((char *)buf, "SOS") == 0) {
        Serial.println(F("SOS detected"));
        if (!isCentralUSV) {
          isCentralUSV = true; // Designate this USV as the central one
        }
        convergeOnSignal();
      }
    }
  }

  // Regularly broadcast status
  broadcastStatus();
}

void convergeOnSignal() {
  while (true) {
    // Listen for data from other USVs using RFM95
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        if (len == sizeof(USVStatus)) {
          USVStatus receivedStatus;
          memcpy(&receivedStatus, buf, sizeof(receivedStatus));
          neighborStatuses[neighborCount] = receivedStatus;
          neighborCount++;
        }
      }
    }

    // If data from enough USVs is received, perform triangulation
    if (neighborCount >= 2) {
      float estimatedLat, estimatedLon;
      if (isCentralUSV) {
        estimateBearing(estimatedLat, estimatedLon);
      } else {
        triangulate(neighborStatuses, neighborCount, estimatedLat, estimatedLon);
      }

      Serial.print(F("ESTIMATED,"));
      Serial.print(estimatedLat, 6);
      Serial.print(F(","));
      Serial.println(estimatedLon, 6);

      // Command USV to move towards estimated position
      moveToTarget(estimatedLat, estimatedLon);

      // Reset neighbor count for the next cycle
      neighborCount = 0;
    }

    // Regularly broadcast status to help neighbors converge
    broadcastStatus();
  }
}

void estimateBearing(float &estimatedLat, float &estimatedLon) {
  // Central USV (Blue) calculates bearing using PDoA
  // Simplified: Use differences in received signal strengths and positions
  float sumX = 0, sumY = 0;
  for (int i = 0; i < neighborCount; i++) {
    float weight = 1.0 / (neighborStatuses[i].signalStrength); // Simple inverse weight
    sumX += (neighborStatuses[i].longitude - myStatus.longitude) * weight;
    sumY += (neighborStatuses[i].latitude - myStatus.latitude) * weight;
  }
  estimatedLat = myStatus.latitude + (sumY / neighborCount);
  estimatedLon = myStatus.longitude + (sumX / neighborCount);
}

void triangulate(USVStatus *statuses, int count, float &estimatedLat, float &estimatedLon) {
  // Implement enhanced triangulation using multiple USVs
  float totalWeight = 0;
  float weightedLat = 0;
  float weightedLon = 0;

  for (int i = 0; i < count; i++) {
    float weight = 1.0 / (statuses[i].signalStrength); // Example weight calculation
    weightedLat += statuses[i].latitude * weight;
    weightedLon += statuses[i].longitude * weight;
    totalWeight += weight;
  }

  estimatedLat = weightedLat / totalWeight;
  estimatedLon = weightedLon / totalWeight;
}

void moveToTarget(float targetLat, float targetLon) {
  // Move USV to the target position using some navigation algorithm
  // You can integrate motor control logic here
  Serial.print(F("Moving to target: "));
  Serial.print(targetLat, 6);
  Serial.print(F(", "));
  Serial.println(targetLon, 6);
}
