// ============================================================
// UAV — Arduino Nano
// ============================================================
// Tar emot data via RS485 (MAX485) från bojen
// Avkodar PWM-värden och GPS-data
// Styr servon och ESC via PCA9685 (I²C)
// ============================================================
// Koppling:
//   D0 (TX)  → MAX485 DI
//   D1 (RX)  ← MAX485 RO
//   D2       → MAX485 DE + RE (ihopkopplade)
//   A4 (SDA) ↔ PCA9685 SDA
//   A5 (SCL) → PCA9685 SCL
// ============================================================
// PCA9685 kanaler:
//   CH0 = ESC (Speed)
//   CH1 = Servo Rudder (Yaw)
//   CH2 = Servo Pitch Port
//   CH3 = Servo Pitch Starbord
//   CH4 = Ballast
// ============================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---- Pin-definitioner ----
#define RS485_DIR     2       // DE + RE på MAX485

// ---- PCA9685 kanaler ----
#define CH_SPEED      0       // ESC
#define CH_YAW        1       // Rudder servo
#define CH_PITCH_P    2       // Pitch port servo
#define CH_PITCH_S    3       // Pitch starbord servo
#define CH_BALLAST    4       // Ballast

#define NUM_CHANNELS  5

// ---- Konfiguration ----
#define RS485_BAUD    115200
#define PCA9685_FREQ  50      // 50 Hz för servon och ESC
#define FAILSAFE_MS   500     // ms utan data → failsafe

// ---- PWM-omvandling ----
// PCA9685 vid 50Hz: 1 tick = 1000000 / (50 * 4096) = ~4.88 µs
// Servo 1000µs = ~205 ticks, 1500µs = ~307 ticks, 2000µs = ~410 ticks
#define US_TO_TICKS(us)  ((uint16_t)((us) * 4096L / 20000L))

// ---- Failsafe-värden (neutral) ----
const uint16_t FAILSAFE_VALUES[NUM_CHANNELS] = {
  1500,   // CH0: Speed → neutral (motor stopp)
  1500,   // CH1: Yaw → rak kurs
  1500,   // CH2: Pitch port → neutral
  1500,   // CH3: Pitch starbord → neutral
  1500    // CH4: Ballast → neutral
};

// ---- Objekt ----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---- Mottagna värden ----
uint16_t channelValues[NUM_CHANNELS];
bool     channelActive = false;

// ---- Parser ----
char     rxBuf[160];
uint8_t  rxIdx = 0;
bool     rxInPacket = false;

// ---- GPS-data (senast mottagen) ----
char     lastGGA[120] = "";
char     lastRMC[120] = "";

// ---- Timing ----
unsigned long lastDataTime = 0;
bool          failsafeActive = false;

// ============================================================
// SETUP
// ============================================================
void setup() {
  // RS485 riktningspin
  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW);   // Default: lyssna (mottagare)

  // Starta serial
  Serial.begin(RS485_BAUD);

  // Starta PCA9685
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(PCA9685_FREQ);

  // Sätt alla kanaler till failsafe vid start
  setFailsafe();

  // Vänta lite så ESC:n initieras (behöver se neutral signal)
  delay(1000);
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // ---- Läs RS485-data ----
  while (Serial.available()) {
    char c = Serial.read();
    parseChar(c);
  }

  // ---- Failsafe-kontroll ----
  if (millis() - lastDataTime > FAILSAFE_MS) {
    if (!failsafeActive) {
      setFailsafe();
      failsafeActive = true;
    }
  }
}

// ============================================================
// Parsa inkommande tecken
// Protokoll: <P:v1,v2,v3,v4,v5,CS> eller <G:$GPGGA,...>
// ============================================================
void parseChar(char c) {
  if (c == '<') {
    // Start av paket
    rxIdx = 0;
    rxInPacket = true;
    return;
  }

  if (c == '>' && rxInPacket) {
    // Slut på paket
    rxBuf[rxIdx] = '\0';
    rxInPacket = false;
    processPacket();
    return;
  }

  if (rxInPacket) {
    if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    } else {
      // Överfull buffert — kassera
      rxInPacket = false;
      rxIdx = 0;
    }
  }
}

// ============================================================
// Bearbeta mottaget paket
// ============================================================
void processPacket() {
  if (rxIdx < 3) return;  // För kort

  if (rxBuf[0] == 'P' && rxBuf[1] == ':') {
    // PWM-paket: P:1500,1500,1500,1500,1500,CS
    processPWMPacket(rxBuf + 2);
  }
  else if (rxBuf[0] == 'G' && rxBuf[1] == ':') {
    // GPS-paket: G:$GPGGA,...  eller  G:$GPRMC,...
    processGPSPacket(rxBuf + 2);
  }
}

// ============================================================
// Bearbeta PWM-paket
// ============================================================
void processPWMPacket(char* data) {
  uint16_t values[NUM_CHANNELS];
  uint8_t  count = 0;
  char*    token;
  char*    rest = data;

  // Parsa kommaseparerade värden
  while ((token = strtok_r(rest, ",", &rest)) != NULL) {
    if (count < NUM_CHANNELS) {
      values[count] = atoi(token);
      count++;
    } else {
      // Sista token = checksumma (hex)
      // Verifiera checksumma
      uint8_t receivedCS = (uint8_t)strtol(token, NULL, 16);
      uint8_t calcCS = 0;

      // Beräkna checksumma över "P:v1,v2,v3,v4,v5"
      // (allt mellan < och ,CS>)
      // Vi behöver räkna om från originaldatan
      char* csEnd = token - 1;  // Peka på kommat före CS
      for (char* p = rxBuf + 1; p < csEnd; p++) {  // Hoppa över buffertens start
        // OBS: rxBuf börjar vid 'P', vi behöver beräkna från 'P'
      }
      // Förenklad: beräkna CS över "P:v1,v2,v3,v4,v5"
      calcCS = 0;
      for (uint8_t i = 0; i < (uint8_t)(csEnd - rxBuf); i++) {
        calcCS ^= rxBuf[i];
      }

      if (calcCS != receivedCS) {
        // Checksummafel — kassera paket
        return;
      }
    }
  }

  if (count == NUM_CHANNELS) {
    // Validera alla värden
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      if (values[i] >= 900 && values[i] <= 2100) {
        channelValues[i] = values[i];
      }
      // Ogiltigt värde → behåll senaste
    }

    channelActive = true;
    failsafeActive = false;
    lastDataTime = millis();

    // Uppdatera servon
    updateServos();
  }
}

// ============================================================
// Bearbeta GPS-paket
// ============================================================
void processGPSPacket(char* data) {
  if (strncmp(data + 3, "GGA", 3) == 0) {
    strncpy(lastGGA, data, sizeof(lastGGA) - 1);
    lastGGA[sizeof(lastGGA) - 1] = '\0';
  }
  else if (strncmp(data + 3, "RMC", 3) == 0) {
    strncpy(lastRMC, data, sizeof(lastRMC) - 1);
    lastRMC[sizeof(lastRMC) - 1] = '\0';
  }

  // GPS-data sparas och kan användas för:
  // - Autonom navigering
  // - Loggning
  // - Geo-fence
  // - Return-to-home
}

// ============================================================
// Uppdatera servon via PCA9685
// ============================================================
void updateServos() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    uint16_t ticks = US_TO_TICKS(channelValues[i]);
    pwm.setPWM(i, 0, ticks);
  }
}

// ============================================================
// Failsafe — sätt alla kanaler till neutral
// ============================================================
void setFailsafe() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    channelValues[i] = FAILSAFE_VALUES[i];
  }
  updateServos();
}

// ============================================================
// Hjälpfunktion: Hämta senaste GPS-position
// Kan anropas från annan kod för autonom navigering
// ============================================================
bool getGPSPosition(float* lat, float* lon) {
  if (strlen(lastGGA) < 20) return false;

  // Parsa NMEA GGA: $GPGGA,hhmmss.ss,ddmm.mmmmm,N,dddmm.mmmmm,E,...
  char* p = lastGGA;

  // Hoppa till latitude-fältet (fält 2)
  for (uint8_t field = 0; field < 2; field++) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
  }

  // Latitude: ddmm.mmmmm
  float rawLat = atof(p);
  int deg = (int)(rawLat / 100);
  float min = rawLat - deg * 100;
  *lat = deg + min / 60.0;

  // N/S
  p = strchr(p, ',');
  if (!p) return false;
  p++;
  if (*p == 'S') *lat = -*lat;

  // Longitude: dddmm.mmmmm
  p = strchr(p, ',');
  if (!p) return false;
  p++;
  float rawLon = atof(p);
  deg = (int)(rawLon / 100);
  min = rawLon - deg * 100;
  *lon = deg + min / 60.0;

  // E/W
  p = strchr(p, ',');
  if (!p) return false;
  p++;
  if (*p == 'W') *lon = -*lon;

  return true;
}
