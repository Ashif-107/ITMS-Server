// Full program: ADXL345 vibration detector + SIM800L SMS control + MFRC522 RFID quick-scan + L298N motor control
// Target MCU: ESP32-WROOM (pins set accordingly)
// Note: preserves original blocks' structure. Adds motor control and SMS "forward" parsing.

#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <EEPROM.h>

// ===================== HARDWARE PIN MAP (ESP32-WROOM) =====================
// I2C (ADXL345)
const int SDA_PIN = 21; // ESP32-WROOM default SDA
const int SCL_PIN = 22; // ESP32-WROOM default SCL
const uint8_t ADXL_ADDR = 0x53;

// SPI (MFRC522)
#define SPI_SCK 18   // SCK
#define SPI_MISO 19  // MISO
#define SPI_MOSI 23  // MOSI
#define SS_PIN   5   // RC522 SS (changed from 23 to 5)
#define RST_PIN  4   // RC522 RST (changed from 20 to 4)
MFRC522 mfrc(SS_PIN, RST_PIN);

// SIM800L serial pins (Serial1)
const int SIM_RX_PIN = 16; // SIM TXD -> ESP RX
const int SIM_TX_PIN = 17; // SIM RXD <- ESP TX
const unsigned long SIM_BAUD = 9600UL;

// L298N motor pins (two motors: A and B)
// Motor A: IN1, IN2, ENA(PWM)
const int M_A_IN1 = 32;
const int M_A_IN2 = 33;
const int M_A_EN  = 25; // PWM

// Motor B: IN3, IN4, ENB(PWM)
const int M_B_IN3 = 26;
const int M_B_IN4 = 27;
const int M_B_EN  = 14; // PWM

// PWM config
const int PWM_FREQ = 1000;
const int PWM_RES_BITS = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// Movement config
const unsigned long MOTOR_FORWARD_MS = 1500UL; // fixed time to move distance. adjust experimentally
const uint8_t MOTOR_PWM_DUTY = 200; // 0-255 (8-bit). adjust speed.

// ===================== EXISTING CONFIG (FFT, RFID, EEPROM, SMS) =====================
const double SAMPLE_RATE = 1000.0;     // Hz (software sample rate)
const uint16_t N = 512;                // FFT size (power of two)
const uint16_t OVERLAP = N / 2;        // overlap
const uint16_t STEP = N - OVERLAP;     // hop size

const double LOW_BAND_HZ = 0.0;
const double LOW_BAND_HZ_END = 50.0;
const double HIGH_BAND_HZ = 50.0;
const double HIGH_BAND_HZ_END = 250.0;

const uint16_t CALIB_SECONDS = 20;     // calibration duration (seconds of processed data)
const uint32_t DEBOUNCE_MS = 200;      // min ms between alerts (FFT-level)
const double R_STD_MULTIPLIER = 4.0;   // mean + R_STD_MULTIPLIER * std  (base)
const double SENSITIVITY_MULT = 1.35;  // final multiplier to reduce sensitivity (~+2%)
const double KURT_THRESH = 11.0;       // kurtosis cutoff fallback
const double RMS_STD_MULTIPLIER = 2.0; // RMS threshold = meanRMS + RMS_STD_MULTIPLIER*stdRMS

// ========== SMS / SIM800L CONFIG ==========
String TARGET_PHONE = "‪+917200545909‬"; // adjustable parameter at top
const unsigned long SMS_INTERVAL_MS = 5000UL; // 5 second buffer between SMS sends

// EEPROM / RFID layout (kept from your code)
const int EEPROM_SIZE = 1024;
const int MAX_CARDS = 50;
const int UID_STORE_MAX = 10;
const int NAME_STORE_MAX = 24;
const int RECORD_SIZE = 1 + UID_STORE_MAX + NAME_STORE_MAX;

// RFID polling timing
unsigned long lastRfidCheckMs = 0;
const unsigned long RFID_POLL_MS = 200; // 200 ms

// ===================== INTERNALS (FFT etc) =====================
double vReal[N];
double vImag[N];
ArduinoFFT FFT = ArduinoFFT(vReal, vImag, N, SAMPLE_RATE);

double sampleBuf[N];               // circular buffer
uint32_t writeIndex = 0;
bool bufFilled = false;
unsigned long lastAlertMs = 0;

// Calibration stats
bool calibrated = false;
double calibSumRatio = 0.0;
double calibSumRatioSq = 0.0;
double calibSumRMS = 0.0;
double calibSumRMSSq = 0.0;
uint32_t calibCount = 0;

double R_thresh = 1.0;
double meanRMS = 0.0;
double stdRMS = 0.0;
double RMS_thresh = 0.0;

// Minimal reporting variables
volatile bool alertFlag = false;
double lastRatio = 0.0;
double lastRMS = 0.0;
double lastKurt = 0.0;

// SMS
unsigned long lastSmsMs = 0;
unsigned long lastStatusMs = 0;
const uint32_t STATUS_PRINT_MS = 1000; // 1 Hz

// Buffer for Serial1 incoming (SIM800L)
String simInBuf = "";

// ===================== ADXL345 I2C helpers (unchanged) =====================
void adxlWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
void adxlReadMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)ADXL_ADDR, (int)len);
  uint8_t i = 0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
}
void adxlInit() {
  adxlWrite(0x31, 0x0B);
  adxlWrite(0x2C, 0x0F);
  adxlWrite(0x2D, 0x08);
  delay(10);
}
double readAccelMagnitude() {
  uint8_t buf[6];
  adxlReadMulti(0x32, buf, 6);
  int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
  const double scale = 0.0039;
  double gx = x * scale;
  double gy = y * scale;
  double gz = z * scale;
  return sqrt(gx*gx + gy*gy + gz*gz);
}

// ===================== Simple stats (unchanged) =====================
double computeMean(const double *arr, uint16_t len) {
  double s = 0;
  for (uint16_t i = 0; i < len; ++i) s += arr[i];
  return s / len;
}
double computeRMS(const double *arr, uint16_t len) {
  double s = 0;
  for (uint16_t i = 0; i < len; ++i) s += arr[i] * arr[i];
  return sqrt(s / len);
}
double computeKurtosis(const double *arr, uint16_t len) {
  double m = computeMean(arr, len);
  double m2 = 0;
  double m4 = 0;
  for (uint16_t i = 0; i < len; ++i) {
    double d = arr[i] - m;
    double d2 = d * d;
    m2 += d2;
    m4 += d2 * d2;
  }
  if (m2 == 0) return 0;
  double var = m2 / len;
  double kurt = (m4 / len) / (var * var);
  return kurt;
}

// ===================== Window processing (unchanged) =====================
void processWindow(const double *timeData) {
  for (uint16_t i = 0; i < N; ++i) { vReal[i] = timeData[i]; vImag[i] = 0.0; }

  double rms = computeRMS(timeData, N);
  double kurt = computeKurtosis(timeData, N);

  FFT.windowing(vReal, N, FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT.compute(vReal, vImag, N, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, N);

  uint16_t maxBin = N / 2;
  double df = SAMPLE_RATE / N;
  uint16_t lowBinStart = (uint16_t)floor(LOW_BAND_HZ / df);
  uint16_t lowBinEnd = (uint16_t)ceil(LOW_BAND_HZ_END / df);
  uint16_t highBinStart = (uint16_t)floor(HIGH_BAND_HZ / df);
  uint16_t highBinEnd = (uint16_t)ceil(HIGH_BAND_HZ_END / df);
  if (lowBinEnd >= maxBin) lowBinEnd = maxBin - 1;
  if (highBinEnd >= maxBin) highBinEnd = maxBin - 1;

  double lowE = 1e-30;
  double highE = 1e-30;
  for (uint16_t k = 1; k < maxBin; ++k) {
    double mag = vReal[k];
    double psd = (mag * mag) / N;
    if (k >= lowBinStart && k <= lowBinEnd) lowE += psd;
    if (k >= highBinStart && k <= highBinEnd) highE += psd;
  }
  double ratio = highE / lowE;

  if (!calibrated) {
    calibSumRatio += ratio;
    calibSumRatioSq += ratio * ratio;
    calibSumRMS += rms;
    calibSumRMSSq += rms * rms;
    calibCount++;

    double elapsedSec = (double)calibCount * (STEP / SAMPLE_RATE);
    Serial.printf("Calib %.1fs ratio=%.4f rms=%.4f kurt=%.4f\n", elapsedSec, ratio, rms, kurt);

    if (elapsedSec >= CALIB_SECONDS) {
      double meanR = calibSumRatio / calibCount;
      double varR = (calibSumRatioSq / calibCount) - (meanR * meanR);
      double stdR = varR > 0 ? sqrt(varR) : 0.0;
      R_thresh = (meanR + R_STD_MULTIPLIER * stdR) * SENSITIVITY_MULT;
      if (R_thresh < 1.0) R_thresh = 1.0;

      meanRMS = calibSumRMS / calibCount;
      double varRMS = (calibSumRMSSq / calibCount) - (meanRMS * meanRMS);
      stdRMS = varRMS > 0 ? sqrt(varRMS) : 0.0;
      RMS_thresh = meanRMS + RMS_STD_MULTIPLIER * stdRMS;

      calibrated = true;
      Serial.printf("Calibration done. R_mean=%.4f R_std=%.4f R_thresh=%.4f meanRMS=%.4f stdRMS=%.4f RMS_thresh=%.4f\n",
                    meanR, stdR, R_thresh, meanRMS, stdRMS, RMS_thresh);
    }
    return;
  }

  bool localAlert = false;
  if ((ratio > R_thresh && rms > RMS_thresh) || (kurt > KURT_THRESH)) {
    unsigned long now = millis();
    if ((now - lastAlertMs) > DEBOUNCE_MS) {
      localAlert = true;
      lastAlertMs = now;
    }
  }

  lastRatio = ratio;
  lastRMS = rms;
  lastKurt = kurt;
  if (localAlert) alertFlag = true;
}

// ===================== SIM800L helpers (unchanged, plus CNMI) =====================
void flushSIM() { while (Serial1.available()) Serial1.read(); }

String readSIMResponse(unsigned long timeout=2000) {
  unsigned long start = millis();
  String res = "";
  while (millis() - start < timeout) {
    while (Serial1.available()) {
      res += char(Serial1.read());
      start = millis();
    }
  }
  return res;
}

bool sendAT(const String &cmd, unsigned long waitMs=1000) {
  Serial1.print(cmd + "\r");
  String r = readSIMResponse(waitMs);
  Serial.print("AT> " + cmd + "\n");
  Serial.print("SIM> " + r + "\n");
  return r.indexOf("OK") != -1;
}

void sendSMS(const String &phone, const String &message) {
  unsigned long now = millis();
  if ((now - lastSmsMs) < SMS_INTERVAL_MS) {
    Serial.println("SMS suppressed by buffer interval.");
    return;
  }
  lastSmsMs = now;

  Serial.println("Sending SMS to " + phone);
  sendAT("AT+CMGF=1");
  String cmd = "AT+CMGS=\"" + phone + "\"";
  Serial1.print(cmd + "\r");

  unsigned long t0 = millis();
  bool prompt = false;
  String acc = "";
  while (millis() - t0 < 4000) {
    while (Serial1.available()) acc += char(Serial1.read());
    if (acc.indexOf(">") != -1) { prompt = true; break; }
  }
  Serial.print("SIM> " + acc + "\n");
  if (!prompt) {
    Serial.println("No '>' prompt — aborting send.");
    return;
  } else {
    Serial1.print(message);
    delay(200);
    Serial1.write(26); // Ctrl+Z
    String resp = readSIMResponse(8000);
    Serial.print("SIM> " + resp + "\n");
    if (resp.indexOf("OK") != -1 || resp.indexOf("+CMGS:") != -1) Serial.println("SMS sent.");
    else Serial.println("SMS may have failed.");
  }
  Serial.println("Done sending SMS.");
}

// ===================== EEPROM / RFID helpers (unchanged) =====================
int numRecords() {
  byte v = EEPROM.read(0);
  if (v > MAX_CARDS) return 0;
  return v;
}
void setNumRecords(byte v) {
  if (v > MAX_CARDS) v = MAX_CARDS;
  EEPROM.write(0, v);
  EEPROM.commit();
}
int recordBase(int index) { return 1 + index * RECORD_SIZE; }

int findRecordByUID(const byte* uid, byte uidlen) {
  int n = numRecords();
  for (int i = 0; i < n; ++i) {
    int base = recordBase(i);
    byte storedLen = EEPROM.read(base);
    if (storedLen == uidlen) {
      bool same = true;
      for (int j = 0; j < uidlen; ++j) {
        byte b = EEPROM.read(base + 1 + j);
        if (b != uid[j]) { same = false; break; }
      }
      if (same) return i;
    }
  }
  return -1;
}
String readNameAtIndex(int index) {
  int base = recordBase(index);
  int nameStart = base + 1 + UID_STORE_MAX;
  char buf[NAME_STORE_MAX + 1];
  for (int i = 0; i <= NAME_STORE_MAX; ++i) buf[i] = 0;
  for (int i = 0; i < NAME_STORE_MAX; ++i) {
    byte v = EEPROM.read(nameStart + i);
    if (v == 0) break;
    buf[i] = (char)v;
  }
  buf[NAME_STORE_MAX] = 0;
  return String(buf);
}
bool storeRecord(const byte* uid, byte uidlen, const char* name) {
  int n = numRecords();
  if (n >= MAX_CARDS) return false;
  int base = recordBase(n);
  EEPROM.write(base, uidlen);
  for (int i = 0; i < UID_STORE_MAX; ++i) {
    byte v = (i < uidlen) ? uid[i] : 0;
    EEPROM.write(base + 1 + i, v);
  }
  int nameLen = 0;
  if (name) nameLen = strlen(name);
  if (nameLen > NAME_STORE_MAX - 1) nameLen = NAME_STORE_MAX - 1;
  for (int i = 0; i < NAME_STORE_MAX; ++i) {
    if (i < nameLen) EEPROM.write(base + 1 + UID_STORE_MAX + i, name[i]);
    else EEPROM.write(base + 1 + UID_STORE_MAX + i, 0);
  }
  setNumRecords(n + 1);
  EEPROM.commit();
  return true;
}

// RC522 helpers
byte readUIDfromMFRC(byte* out, byte maxLen) {
  if (!mfrc.uid.size) return 0;
  byte len = min((byte)mfrc.uid.size, maxLen);
  for (byte i = 0; i < len; ++i) out[i] = mfrc.uid.uidByte[i];
  return len;
}
String uidToStr(const byte* uid, byte len) {
  String s = "";
  for (byte i = 0; i < len; ++i) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
    if (i + 1 < len) s += ":";
  }
  s.toUpperCase();
  return s;
}
bool parseUidHex(const String &hexstr, byte *out, byte &outLen) {
  String s = hexstr; s.replace(":", ""); s.trim();
  if (s.length() % 2 != 0) return false;
  outLen = s.length() / 2;
  for (byte i = 0; i < outLen; ++i) {
    String byteHex = s.substring(i*2, i*2 + 2);
    out[i] = (byte) strtoul(byteHex.c_str(), NULL, 16);
  }
  return true;
}

// ===================== MOTOR CONTROL =====================

void motorsSetupPins() {
  pinMode(M_A_IN1, OUTPUT);
  pinMode(M_A_IN2, OUTPUT);
  pinMode(M_B_IN3, OUTPUT);
  pinMode(M_B_IN4, OUTPUT);
  // PWM channels
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(M_A_EN, PWM_CHANNEL_A);
  ledcAttachPin(M_B_EN, PWM_CHANNEL_B);
  // Ensure stopped
  digitalWrite(M_A_IN1, LOW);
  digitalWrite(M_A_IN2, LOW);
  digitalWrite(M_B_IN3, LOW);
  digitalWrite(M_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void stopMotors() {
  digitalWrite(M_A_IN1, LOW);
  digitalWrite(M_A_IN2, LOW);
  digitalWrite(M_B_IN3, LOW);
  digitalWrite(M_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void driveForward(uint8_t duty) {
  // Motor A forward
  digitalWrite(M_A_IN1, HIGH);
  digitalWrite(M_A_IN2, LOW);
  // Motor B forward
  digitalWrite(M_B_IN3, HIGH);
  digitalWrite(M_B_IN4, LOW);
  // PWM
  ledcWrite(PWM_CHANNEL_A, duty);
  ledcWrite(PWM_CHANNEL_B, duty);
}

void moveForwardFixedDistance() {
  Serial.println("Motor: moving forward fixed distance.");
  driveForward(MOTOR_PWM_DUTY);
  delay(MOTOR_FORWARD_MS);
  stopMotors();
  Serial.println("Motor: move complete.");
}

// ===================== PRELOAD RFID =====================
void preloadKnownRFIDs() {
  if (numRecords() == 0) {
    byte u1[UID_STORE_MAX]; byte l1;
    byte u2[UID_STORE_MAX]; byte l2;
    if (parseUidHex("BD:98:F2:8F", u1, l1)) storeRecord(u1, l1, "Chennai");
    if (parseUidHex("3E:83:9B:AE", u2, l2)) storeRecord(u2, l2, "Bengaluru");
    Serial.println("Preloaded known RFID records into EEPROM.");
  } else {
    Serial.printf("EEPROM already has %d records.\n", numRecords());
  }
}

// ===================== SETUP (initialization) =====================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // ADXL345 I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(50);
  adxlInit();
  Serial.printf("ADXL345 I2C vibration detector starting. SAMPLE_RATE=%.0f Hz, N=%u\n", SAMPLE_RATE, N);

  // Motors
  motorsSetupPins();

  // SIM800L serial
  Serial1.begin(SIM_BAUD, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);
  delay(300);
  Serial.println("\nSIM800L SMS sender (ESP32-WROOM) ready.");
  flushSIM();
  sendAT("AT");
  sendAT("ATE0");
  sendAT("AT+CMGF=1");           // text mode
  sendAT("AT+CSCS=\"GSM\"");     // charset
  // Request message forwarding to UART: new SMS delivered directly to UART for parsing
  sendAT("AT+CNMI=2,2,0,0,0");

  // EEPROM init and RFID preload
  EEPROM.begin(EEPROM_SIZE);
  preloadKnownRFIDs();

  // MFRC522 (SPI) init: SPI pins set by SPI.begin(sck, miso, mosi)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  mfrc.PCD_Init();
  Serial.println("RFID reader initialised. Polling enabled.");
}

// ===================== LOOP (main) =====================
void loop() {
  static uint32_t samplesCollected = 0;
  static int64_t nextUs = esp_timer_get_time();

  double mag = readAccelMagnitude();
  sampleBuf[writeIndex] = mag;
  writeIndex = (writeIndex + 1) % N;
  samplesCollected++;

  if (!bufFilled && samplesCollected >= N) bufFilled = true;

  if (bufFilled && (samplesCollected % STEP == 0)) {
    double window[N];
    for (uint16_t i = 0; i < N; ++i) {
      uint32_t p = (writeIndex + i) % N; // oldest -> newest
      window[i] = sampleBuf[p];
    }
    processWindow(window);
  }

  unsigned long nowMs = millis();
  if ((nowMs - lastStatusMs) >= STATUS_PRINT_MS) {
    lastStatusMs = nowMs;
    if (!calibrated) {
      Serial.printf("Calibrating... elapsed %.1f s\n", (double)calibCount * (STEP / SAMPLE_RATE));
    } else {
      Serial.printf("R=%.3f RMS=%.4f Kurt=%.2f Rth=%.3f RMSth=%.4f\n",
                    lastRatio, lastRMS, lastKurt, R_thresh, RMS_thresh);
      if (alertFlag) {
        Serial.println("!!! ALERT: impulsive/high-frequency vibration detected !!!");
        String body = "ALERT: vibration. R=" + String(lastRatio,3) +
                      " RMS=" + String(lastRMS,4) +
                      " Kurt=" + String(lastKurt,2);
        sendSMS(TARGET_PHONE, body);
        alertFlag = false;
      }
    }
  }

  // ---------------- RFID poll (non-blocking) ----------------
  if ((nowMs - lastRfidCheckMs) >= RFID_POLL_MS) {
    lastRfidCheckMs = nowMs;
    if (mfrc.PICC_IsNewCardPresent() && mfrc.PICC_ReadCardSerial()) {
      byte uid[UID_STORE_MAX];
      byte len = readUIDfromMFRC(uid, UID_STORE_MAX);
      if (len > 0) {
        int idx = findRecordByUID(uid, len);
        String uidS = uidToStr(uid, len);
        if (idx >= 0) {
          String name = readNameAtIndex(idx);
          Serial.print("Successfully passed \"RFID ");
          Serial.print(name);
          Serial.print(" ");
          Serial.print(uidS);
          Serial.println("\"");
          // SMS notifying pass
          String passMsg = "Passed " + name;
          sendSMS(TARGET_PHONE, passMsg);
        } else {
          Serial.print("Successfully passed \"RFID Unknown ");
          Serial.print(uidS);
          Serial.println("\"");
          sendSMS(TARGET_PHONE, "Passed Unknown");
        }
      }
      mfrc.PICC_HaltA();
    }
  }

  // ---------------- SIM800L incoming parsing ----------------
  // We set AT+CNMI=2,2 so new SMS texts are pushed to UART.
  // Read Serial1 and collect into simInBuf. When a newline arrives, check for keyword.
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    simInBuf += c;
    // limit buffer to reasonable size
    if (simInBuf.length() > 1024) simInBuf = simInBuf.substring(simInBuf.length() - 1024);
    // process on newline
    if (c == '\n' || c == '\r') {
      String ln = simInBuf;
      ln.trim();
      if (ln.length() > 0) {
        String ll = ln;
        ll.toLowerCase();
        // check for 'forward' word in incoming message payload
        if (ll.indexOf("forward") != -1) {
          Serial.println("SMS command detected: FORWARD -> moving cart.");
          sendSMS(TARGET_PHONE, "Moving forward.");
          moveForwardFixedDistance();
        }
        // optional other commands here (backward, stop) can be added similarly
      }
      simInBuf = "";
    }
  }

  // ---------------- timing enforcement for sampling ----------------
  nextUs += (int64_t)(1e6 / SAMPLE_RATE);
  int64_t nowUs;
  while ((nowUs = esp_timer_get_time()) < nextUs) {
    yield();
  }
}