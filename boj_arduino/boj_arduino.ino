// ============================================================
// BOJ — Arduino Nano
// ============================================================
// Läser PWM från Skyfly 2.4GHz mottagare (5 kanaler)
// Läser GPS NMEA via SoftwareSerial (MAX3232)
// Skickar allt via RS485 (MAX485) till UAV
// ============================================================
// Koppling:
//   D0 (TX)  → MAX485 DI
//   D1 (RX)  ← MAX485 RO
//   D2       → MAX485 DE + RE (ihopkopplade)
//   D4       ← MAX3232 TTL TX (GPS SoftSerial RX)
//   D5       → MAX3232 TTL RX (GPS SoftSerial TX)
//   D6       ← Skyfly CH1 (Speed)
//   D7       ← Skyfly CH2 (Yaw)
//   D8       ← Skyfly CH3 (Pitch Port)
//   D9       ← Skyfly CH4 (Pitch Starbord)
//   D10      ← Skyfly CH5 (Ballast)
// ============================================================

#include <SoftwareSerial.h>

// ---- Pin-definitioner ----
#define RS485_DIR     2       // DE + RE på MAX485
#define GPS_RX        4       // ← MAX3232 TTL TX
#define GPS_TX        5       // → MAX3232 TTL RX

#define PWM_CH1       6       // Speed +/−
#define PWM_CH2       7       // Yaw port/starbord
#define PWM_CH3       8       // Pitch port
#define PWM_CH4       9       // Pitch starbord
#define PWM_CH5       10      // Ballast +/−

#define NUM_CHANNELS  5

// ---- Konfiguration ----
#define RS485_BAUD    115200
#define GPS_BAUD      9600
#define PWM_TIMEOUT   25000   // µs timeout för pulseIn
#define SEND_INTERVAL 20      // ms mellan sändningar (~50 Hz)
#define GPS_INTERVAL  200     // ms mellan GPS-sändningar (~5 Hz)

// ---- Objekt ----
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// ---- PWM-kanaler ----
const uint8_t pwmPins[NUM_CHANNELS] = { PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4, PWM_CH5 };
uint16_t pwmValues[NUM_CHANNELS];

// ---- GPS-buffert ----
char gpsBuf[120];
uint8_t gpsIdx = 0;
bool gpsReady = false;

// ---- Timing ----
unsigned long lastPwmSend = 0;
unsigned long lastGpsSend = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  // RS485 riktningspin
  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW);   // Default: lyssna

  // PWM-ingångar
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pinMode(pwmPins[i], INPUT);
  }

  // Starta serial
  Serial.begin(RS485_BAUD);       // Hårdvaru-serial → MAX485
  gpsSerial.begin(GPS_BAUD);      // SoftwareSerial → GPS via MAX3232
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // ---- Läs PWM och skicka ----
  if (now - lastPwmSend >= SEND_INTERVAL) {
    lastPwmSend = now;
    readAllPWM();
    sendPWMPacket();
  }

  // ---- Läs GPS kontinuerligt ----
  readGPS();

  // ---- Skicka GPS-data med lägre frekvens ----
  if (gpsReady && (now - lastGpsSend >= GPS_INTERVAL)) {
    lastGpsSend = now;
    sendGPSPacket();
    gpsReady = false;
  }
}

// ============================================================
// Läs alla PWM-kanaler
// ============================================================
void readAllPWM() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    unsigned long pw = pulseIn(pwmPins[i], HIGH, PWM_TIMEOUT);
    if (pw == 0) {
      // Ingen signal — behåll senaste värdet eller sätt neutral
      pwmValues[i] = 1500;  // Neutral/failsafe
    } else {
      // Begränsa till giltigt RC-intervall
      pwmValues[i] = constrain(pw, 900, 2100);
    }
  }
}

// ============================================================
// Skicka PWM-paket via RS485
// Format: <P:1500,1500,1500,1500,1500,CS>
// CS = enkel checksumma (XOR av alla bytes mellan < och ,CS>)
// ============================================================
void sendPWMPacket() {
  char packet[60];
  uint8_t len = snprintf(packet, sizeof(packet), "<P:%u,%u,%u,%u,%u",
                         pwmValues[0], pwmValues[1], pwmValues[2],
                         pwmValues[3], pwmValues[4]);

  // Beräkna checksumma (XOR av payload)
  uint8_t cs = 0;
  for (uint8_t i = 1; i < len; i++) {  // Hoppa över '<'
    cs ^= packet[i];
  }

  // Lägg till checksumma och avslutning
  len += snprintf(packet + len, sizeof(packet) - len, ",%02X>", cs);

  rs485Send(packet, len);
}

// ============================================================
// Läs GPS NMEA-data (icke-blockerande)
// ============================================================
void readGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();

    if (c == '$') {
      // Start av ny NMEA-mening
      gpsIdx = 0;
      gpsBuf[gpsIdx++] = c;
    } else if (c == '\n' || c == '\r') {
      // Slut på NMEA-mening
      if (gpsIdx > 6) {
        gpsBuf[gpsIdx] = '\0';

        // Filtrera — skicka bara GGA (position) och RMC (kurs/hastighet)
        if (strncmp(gpsBuf + 3, "GGA", 3) == 0 ||
            strncmp(gpsBuf + 3, "RMC", 3) == 0) {
          gpsReady = true;
        }
      }
      gpsIdx = 0;
    } else {
      if (gpsIdx < sizeof(gpsBuf) - 1) {
        gpsBuf[gpsIdx++] = c;
      }
    }
  }
}

// ============================================================
// Skicka GPS-paket via RS485
// Format: <G:$GPGGA,123456.00,...>
// ============================================================
void sendGPSPacket() {
  char packet[140];
  uint8_t len = snprintf(packet, sizeof(packet), "<G:%s>", gpsBuf);
  rs485Send(packet, len);
}

// ============================================================
// RS485 sändningsfunktion
// ============================================================
void rs485Send(const char* data, uint8_t len) {
  digitalWrite(RS485_DIR, HIGH);    // Aktivera sändning
  delayMicroseconds(100);           // Låt transceivern stabilisera
  Serial.write((const uint8_t*)data, len);
  Serial.flush();                   // Vänta tills allt skickats
  delayMicroseconds(100);           // Extra marginal
  digitalWrite(RS485_DIR, LOW);     // Tillbaka till mottagning
}
