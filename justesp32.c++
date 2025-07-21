#include <WiFi.h>
#include <base64.h>

#define WIFI_SSID      "4 zero one"
#define WIFI_PASSWORD  "mukandagumbo"

#define NTRIP_SERVER   "rtk2go.com"
#define NTRIP_PORT     2101
#define MOUNT_POINT    "Renewables"
#define NTRIP_USER     "s229701698@mandela.ac.za"
#define NTRIP_PASS     "mukandagumbo"

WiFiClient ntrip;

unsigned long lastCountMs = 0;
unsigned long lastStatsMs = 0;
size_t byteCount = 0;

// RTCM3 parsing variables
uint8_t rtcmBuffer[1024];
int bufferIndex = 0;
bool inMessage = false;

// Message counters
// Replace your current message counter declarations with this complete set:
// Message counters
int msgCount1001 = 0, msgCount1002 = 0, msgCount1003 = 0, msgCount1004 = 0;
int msgCount1005 = 0, msgCount1006 = 0, msgCount1007 = 0, msgCount1008 = 0;
int msgCount1009 = 0, msgCount1010 = 0, msgCount1011 = 0, msgCount1012 = 0;
int msgCount1019 = 0, msgCount1020 = 0, msgCount1074 = 0, msgCount1075 = 0;
int msgCount1077 = 0, msgCount1084 = 0, msgCount1085 = 0, msgCount1087 = 0;
int msgCount1094 = 0, msgCount1095 = 0, msgCount1097 = 0, msgCount1124 = 0;
int msgCount1125 = 0, msgCount1127 = 0, msgCount1230 = 0, msgCountOther = 0;

// Satellite tracking
struct SatInfo {
  int prn;
  int constellation; // 0=GPS, 1=GLO, 2=GAL, 3=BDS
  unsigned long lastSeen;
};
SatInfo satellites[64];
int satCount = 0;

const char* getConstellationName(int constellation) {
  switch(constellation) {
    case 0: return "GPS";
    case 1: return "GLO";
    case 2: return "GAL";
    case 3: return "BDS";
    default: return "UNK";
  }
}

const char* getMessageName(int msgType) {
  switch(msgType) {
    case 1001: return "GPS L1 RTK";
    case 1002: return "GPS L1 RTK Ext";
    case 1003: return "GPS L1&L2 RTK";
    case 1004: return "GPS L1&L2 RTK Ext";
    case 1005: return "Station ARP";
    case 1006: return "Station ARP+Height";
    case 1007: return "Antenna Desc";
    case 1008: return "Antenna Desc+Serial";
    case 1009: return "GLONASS L1 RTK";
    case 1010: return "GLONASS L1 RTK Ext";
    case 1011: return "GLONASS L1&L2 RTK";
    case 1012: return "GLONASS L1&L2 RTK Ext";
    case 1019: return "GPS Ephemeris";
    case 1020: return "GLONASS Ephemeris";
    case 1074: return "GPS MSM4";
    case 1075: return "GPS MSM5";
    case 1077: return "GPS MSM7";
    case 1084: return "GLONASS MSM4";
    case 1085: return "GLONASS MSM5";
    case 1087: return "GLONASS MSM7";
    case 1094: return "Galileo MSM4";
    case 1095: return "Galileo MSM5";
    case 1097: return "Galileo MSM7";
    case 1124: return "BeiDou MSM4";
    case 1125: return "BeiDou MSM5";
    case 1127: return "BeiDou MSM7";
    case 1230: return "GLONASS L1/L2 Bias";
    default: return "Unknown";
  }
}

void updateSatellite(int prn, int constellation) {
  // Find existing satellite or add new one
  for(int i = 0; i < satCount; i++) {
    if(satellites[i].prn == prn && satellites[i].constellation == constellation) {
      satellites[i].lastSeen = millis();
      return;
    }
  }
  
  // Add new satellite if space available
  if(satCount < 64) {
    satellites[satCount].prn = prn;
    satellites[satCount].constellation = constellation;
    satellites[satCount].lastSeen = millis();
    satCount++;
  }
}

void cleanOldSatellites() {
  unsigned long now = millis();
  int writeIndex = 0;
  
  for(int i = 0; i < satCount; i++) {
    if(now - satellites[i].lastSeen < 60000) { // Keep satellites seen in last 60s
      if(writeIndex != i) {
        satellites[writeIndex] = satellites[i];
      }
      writeIndex++;
    }
  }
  satCount = writeIndex;
}

void parseRTCMMessage(uint8_t* msg, int length) {
  if(length < 6) return;
  
  // Extract message type (12 bits after preamble and reserved bits)
  int msgType = ((msg[3] & 0xFF) << 4) | ((msg[4] & 0xF0) >> 4);
  
  // Count messages
  switch(msgType) {
    case 1001: msgCount1001++; break;
    case 1002: msgCount1002++; break;
    case 1003: msgCount1003++; break;
    case 1004: msgCount1004++; break;
    case 1005: msgCount1005++; break;
    case 1006: msgCount1006++; break;
    case 1007: msgCount1007++; break;
    case 1008: msgCount1008++; break;
    case 1009: msgCount1009++; break;
    case 1010: msgCount1010++; break;
    case 1011: msgCount1011++; break;
    case 1012: msgCount1012++; break;
    case 1019: msgCount1019++; break;
    case 1020: msgCount1020++; break;
    case 1074: msgCount1074++; break;
    case 1075: msgCount1075++; break;
    case 1077: msgCount1077++; break;
    case 1084: msgCount1084++; break;
    case 1085: msgCount1085++; break;
    case 1087: msgCount1087++; break;
    case 1094: msgCount1094++; break;
    case 1095: msgCount1095++; break;
    case 1097: msgCount1097++; break;
    case 1124: msgCount1124++; break;
    case 1125: msgCount1125++; break;
    case 1127: msgCount1127++; break;
    case 1230: msgCount1230++; break;
    default: msgCountOther++; break;
  }
  
  Serial.printf("[MSG %04d] %s (%d bytes)\n", msgType, getMessageName(msgType), length);
  
  // Parse satellite data from RTK messages
  if(msgType >= 1001 && msgType <= 1004) {
    // GPS RTK messages
    if(length >= 14) {
      int numSats = (msg[12] & 0x1F); // Number of satellites (5 bits)
      Serial.printf("  GPS Satellites: %d\n", numSats);
      for(int i = 0; i < numSats && i < 12; i++) {
        updateSatellite(i + 1, 0); // GPS constellation
      }
    }
  }
  else if(msgType >= 1009 && msgType <= 1012) {
    // GLONASS RTK messages
    if(length >= 14) {
      int numSats = (msg[12] & 0x1F); // Number of satellites
      Serial.printf("  GLONASS Satellites: %d\n", numSats);
      for(int i = 0; i < numSats && i < 12; i++) {
        updateSatellite(i + 1, 1); // GLONASS constellation
      }
    }
  }
  else if(msgType == 1074) {
    // GPS MSM4 - more detailed satellite info
    if(length >= 20) {
      // MSM header contains satellite mask (64 bits)
      uint64_t satMask = 0;
      for(int i = 0; i < 8; i++) {
        satMask |= ((uint64_t)msg[13 + i] << (56 - i * 8));
      }
      int satCount = 0;
      for(int i = 0; i < 64; i++) {
        if(satMask & (1ULL << (63 - i))) {
          updateSatellite(i + 1, 0); // GPS PRN 1-32
          satCount++;
        }
      }
      Serial.printf("  GPS MSM4 Satellites: %d\n", satCount);
    }
  }
  else if(msgType == 1077 || msgType == 1087 || msgType == 1097 || msgType == 1127) {
    // MSM7 messages - highest precision
    const char* constel = "UNK";
    int constelId = 0;
    if(msgType == 1077) { constel = "GPS"; constelId = 0; }
    else if(msgType == 1087) { constel = "GLONASS"; constelId = 1; }
    else if(msgType == 1097) { constel = "Galileo"; constelId = 2; }
    else if(msgType == 1127) { constel = "BeiDou"; constelId = 3; }
    
    if(length >= 20) {
      // Parse satellite mask from MSM7 header
      uint64_t satMask = 0;
      for(int i = 0; i < 8; i++) {
        satMask |= ((uint64_t)msg[13 + i] << (56 - i * 8));
      }
      int satCount = 0;
      for(int i = 0; i < 64; i++) {
        if(satMask & (1ULL << (63 - i))) {
          updateSatellite(i + 1, constelId);
          satCount++;
        }
      }
      Serial.printf("  %s MSM7 Satellites: %d (High Precision)\n", constel, satCount);
    }
  }
  else if(msgType == 1075 || msgType == 1085 || msgType == 1095 || msgType == 1125) {
    // MSM5 messages - medium precision
    const char* constel = "UNK";
    if(msgType == 1075) constel = "GPS";
    else if(msgType == 1085) constel = "GLONASS";
    else if(msgType == 1095) constel = "Galileo";
    else if(msgType == 1125) constel = "BeiDou";
    Serial.printf("  %s MSM5 message (Medium Precision)\n", constel);
  }
  else if(msgType == 1230) {
    // GLONASS code-phase bias
    Serial.println("  GLONASS L1/L2 Code-Phase Bias Correction");
  }
}

void processRTCMByte(uint8_t byte) {
  if(!inMessage) {
    // Look for RTCM3 preamble (0xD3)
    if(byte == 0xD3) {
      rtcmBuffer[0] = byte;
      bufferIndex = 1;
      inMessage = true;
    }
  } else {
    rtcmBuffer[bufferIndex++] = byte;
    
    if(bufferIndex >= 3) {
      // Get message length from header
      int msgLength = ((rtcmBuffer[1] & 0x03) << 8) | rtcmBuffer[2];
      int totalLength = msgLength + 6; // Header (3) + Data + CRC (3)
      
      if(bufferIndex >= totalLength) {
        // Complete message received
        parseRTCMMessage(rtcmBuffer, totalLength);
        inMessage = false;
        bufferIndex = 0;
      } else if(bufferIndex >= 1024) {
        // Buffer overflow, reset
        inMessage = false;
        bufferIndex = 0;
      }
    }
  }
}

void printStatistics() {
  Serial.println("\n=== RTCM STATISTICS ===");
  Serial.printf("GPS RTK: 1001:%d 1002:%d 1003:%d 1004:%d\n", 
                msgCount1001, msgCount1002, msgCount1003, msgCount1004);
  Serial.printf("GLO RTK: 1009:%d 1010:%d 1011:%d 1012:%d\n", 
                msgCount1009, msgCount1010, msgCount1011, msgCount1012);
  Serial.printf("Station: 1005:%d 1006:%d 1007:%d 1008:%d\n", 
                msgCount1005, msgCount1006, msgCount1007, msgCount1008);
  Serial.printf("Ephemeris: GPS:%d GLO:%d\n", msgCount1019, msgCount1020);
  Serial.printf("GPS MSM: 4:%d 5:%d 7:%d\n", msgCount1074, msgCount1075, msgCount1077);
  Serial.printf("GLO MSM: 4:%d 5:%d 7:%d Bias:%d\n", msgCount1084, msgCount1085, msgCount1087, msgCount1230);
  Serial.printf("GAL MSM: 4:%d 5:%d 7:%d\n", msgCount1094, msgCount1095, msgCount1097);
  Serial.printf("BDS MSM: 4:%d 5:%d 7:%d\n", msgCount1124, msgCount1125, msgCount1127);
  Serial.printf("Other: %d\n", msgCountOther);
  
  cleanOldSatellites();
  Serial.printf("\n=== ACTIVE SATELLITES (%d) ===\n", satCount);
  
  int gpsCount = 0, gloCount = 0, galCount = 0, bdsCount = 0;
  for(int i = 0; i < satCount; i++) {
    Serial.printf("%s%02d ", getConstellationName(satellites[i].constellation), 
                  satellites[i].prn);
    switch(satellites[i].constellation) {
      case 0: gpsCount++; break;
      case 1: gloCount++; break;
      case 2: galCount++; break;
      case 3: bdsCount++; break;
    }
    if((i + 1) % 8 == 0) Serial.println();
  }
  if(satCount % 8 != 0) Serial.println();
  
  Serial.printf("GPS:%d GLO:%d GAL:%d BDS:%d\n", gpsCount, gloCount, galCount, bdsCount);
  Serial.println("=======================\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- RTCM Message Decoder ---");

  // Connect Wi‑Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi‑Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n✔ Wi‑Fi connected! IP: " + WiFi.localIP().toString());

  // Connect to caster
  Serial.printf("Connecting to %s:%d …\n", NTRIP_SERVER, NTRIP_PORT);
  if (!ntrip.connect(NTRIP_SERVER, NTRIP_PORT)) {
    Serial.println("✗ Failed to connect to caster");
    while (true) delay(1000);
  }

  // Send NTRIP request
  String auth = base64::encode(String(NTRIP_USER) + ":" + String(NTRIP_PASS));
  String req  = 
    "GET /" + String(MOUNT_POINT) + " HTTP/1.1\r\n" +
    "Host: " + NTRIP_SERVER + "\r\n" +
    "User-Agent: ESP32-NTRIP/1.0\r\n" +
    "Authorization: Basic " + auth + "\r\n" +
    "Connection: close\r\n\r\n";
  ntrip.print(req);

  // Wait for "200 OK"
  unsigned long t0 = millis();
  while (!ntrip.available() && millis() - t0 < 5000) {
    delay(10);
  }
  if (ntrip.available()) {
    String line = ntrip.readStringUntil('\n');
    if (line.indexOf("200 OK") < 0) {
      Serial.println("✗ Bad response: " + line);
      while (true) delay(1000);
    }
    Serial.println("✔ NTRIP connected, parsing RTCM messages…");
    // Skip remaining headers
    while (ntrip.available() && ntrip.readStringUntil('\n') != "\r") {}
  } else {
    Serial.println("✗ No response from caster");
    while (true) delay(1000);
  }

  lastCountMs = millis();
  lastStatsMs = millis();
}

void loop() {
  // Process incoming bytes
  while (ntrip.available()) {
    uint8_t b = ntrip.read();
    processRTCMByte(b);
    byteCount++;
  }

  // Print byte count every second
  if (millis() - lastCountMs >= 1000) {
    Serial.printf("Bytes/sec: %u\n", byteCount);
    byteCount = 0;
    lastCountMs += 1000;
  }
  
  // Print detailed statistics every 30 seconds
  if (millis() - lastStatsMs >= 30000) {
    printStatistics();
    lastStatsMs += 1000;
  }
}
