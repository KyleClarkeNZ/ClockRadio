// CJMCU-470 (Si4703) FM Radio - Tune to 91.8 MHz  +  ST7789 170x320 display
//
// Wiring - Si4703:
//   CJMCU-470 VCC  -> Arduino 3.3V
//   CJMCU-470 GND  -> Arduino GND
//   CJMCU-470 SDA  -> Arduino SDA (pin 2)
//   CJMCU-470 SCL  -> Arduino SCL (pin 3)
//   CJMCU-470 RST  -> Arduino pin 7
//
// Wiring - ST7789 170x320 (hardware SPI):
//   ST7789 GND  -> Arduino GND
//   ST7789 VCC  -> Arduino 5V
//   ST7789 SCL  -> Arduino pin 15 (SCK)
//   ST7789 SDA  -> Arduino pin 16 (MOSI)
//   ST7789 RES  -> Arduino pin 4
//   ST7789 DC   -> Arduino pin 5
//   ST7789 CS   -> Arduino pin 6
//   ST7789 BLK  -> Arduino 5V (backlight always on)
//
// Wiring - DS3231 RTC (shared I2C bus):
//   DS3231 VCC  -> Arduino 3.3V (or 5V)
//   DS3231 GND  -> Arduino GND
//   DS3231 SDA  -> Arduino SDA (pin 2)
//   DS3231 SCL  -> Arduino SCL (pin 3)
//
// Audio output is on the CJMCU-470 onboard 3.5mm headphone jack.

#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#define SI4703_ADDR   0x10
#define SI4703_RST    7       // Pin 14 conflicts with SPI MISO — moved to pin 7

// ST7789 display pins (hardware SPI: SCK=15, MOSI=16 used automatically)
#define TFT_CS    6
#define TFT_DC    5
#define TFT_RST   4

// Target frequency in MHz (87.5–108.0, 0.1 MHz steps)
#define TARGET_FREQ   91.8f

// Register addresses
#define REG_POWERCFG   0x02
#define REG_CHANNEL    0x03
#define REG_SYSCONFIG1 0x04
#define REG_SYSCONFIG2 0x05
#define REG_TEST1      0x07
#define REG_STATUSRSSI 0x0A
#define REG_READCHAN   0x0B

// POWERCFG bits
#define POWERCFG_DSMUTE  0x8000  // Disable softmute
#define POWERCFG_DMUTE   0x4000  // Disable mute (enable audio)
#define POWERCFG_ENABLE  0x0001  // Power up

// TEST1 bits
#define TEST1_XOSCEN     0x8000  // Enable crystal oscillator

// CHANNEL bits
#define CHANNEL_TUNE     0x8000  // Start tune operation

// STATUSRSSI bits
#define STATUS_STC       0x4000  // Seek/Tune Complete
#define STATUS_ST        0x0100  // Stereo indicator

// SYSCONFIG1 bits
#define SYSCONFIG1_DE    0x0800  // De-emphasis: 1=50µs (Europe), 0=75µs (USA)

#define LIGHTBLUE  0x5DDF        // light-blue from reference clock display

uint16_t regs[16];

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

RTC_DS3231 rtc;
bool       g_rtc_ok = false;

// Results shared between setup() and loop()
float    g_freq   = 0;
uint8_t  g_rssi   = 0;
bool     g_stereo = false;
bool     g_ok     = false;

// ── UI state machine ──────────────────────────────────────────────────────
enum UIState { UI_CLOCK, UI_VOLUME, UI_STATION };
UIState       g_uiState   = UI_CLOCK;
UIState       g_prevState = (UIState)255;   // force first-entry redraw
unsigned long g_lastActivity = 0;
const unsigned long IDLE_TIMEOUT = 3000;     // ms of stillness -> back to clock

#define STATION_POT  A1    // A1 selects station
#define VOLUME_POT   A0    // A0 sets volume
const int POT_MOVE_THRESH = 8;
int  g_lastStationRaw = 0;

// Top 5 Auckland FM stations (MHz)
const float STATIONS[5] = { 88.6f, 91.8f, 94.2f, 97.8f, 99.0f };
int  g_station      = 1;   // index into STATIONS (default 91.8)
int  g_shownStation = -1;  // last station drawn
int  g_shownMinute  = -1;  // last minute drawn
int  g_shownVol     = -1;  // last volume drawn
int  g_volume       =  8;  // current Si4703 volume (0-15)
int  g_volEma       =  0;  // EMA-filtered volume pot (0-1023); updated every loop tick

// Returns true (and updates stored hash) only when a new sketch has been
// uploaded — power cycles leave the hash unchanged so the RTC is not touched.
static bool isNewUpload() {
  // Hash of compile-time __DATE__ + __TIME__ strings stored at EEPROM addr 0
  uint16_t hash = 0;
  for (const char* p = __DATE__; *p; p++) hash = (uint16_t)(hash * 31u + (uint8_t)*p);
  for (const char* p = __TIME__; *p; p++) hash = (uint16_t)(hash * 31u + (uint8_t)*p);
  uint16_t stored;
  EEPROM.get(0, stored);
  if (stored != hash) {
    EEPROM.put(0, hash);
    return true;
  }
  return false;
}

// Si4703 reads always start at register 0x0A and wrap to 0x00
void readRegisters() {
  Wire.requestFrom((uint8_t)SI4703_ADDR, (uint8_t)32);
  for (int i = 0x0A; i <= 0x0F; i++)
    regs[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
  for (int i = 0x00; i <= 0x09; i++)
    regs[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
}

// Si4703 writes always start at register 0x02
void writeRegisters() {
  Wire.beginTransmission(SI4703_ADDR);
  for (int i = 0x02; i <= 0x07; i++) {
    Wire.write(regs[i] >> 8);
    Wire.write(regs[i] & 0xFF);
  }
  Wire.endTransmission();
}

void setup() {
  Serial.begin(9600);
  delay(200);

  // ── Display init ─────────────────────────────────────
  tft.init(170, 320);
  tft.setRotation(1);            // landscape: 320×170
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(100, 8);
  tft.print(F("FM RADIO"));
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(110, 30);
  tft.print(F("Initialising..."));

  // ── I2C bus (RST pulse first so Si4703 is in known state) ─
  pinMode(SI4703_RST, OUTPUT);
  digitalWrite(SI4703_RST, LOW);
  delay(10);
  digitalWrite(SI4703_RST, HIGH);
  delay(10);
  Wire.begin();
  Wire.setWireTimeout(25000, false);

  // ── Step 0: RTC — set time only on new upload or if battery died ────────
  Serial.println(F("[0] RTC init..."));
  if (!rtc.begin()) {
    Serial.println(F("    Not found — clock display disabled"));
  } else {
    bool newUpload  = isNewUpload();
    bool lostPower  = rtc.lostPower();
    if (newUpload || lostPower) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.print(F("    Time set (reason: "));
      Serial.print(newUpload ? F("new upload") : F("battery lost"));
      Serial.print(F(") -> "));
    } else {
      Serial.print(F("    Keeping RTC time: "));
    }
    g_rtc_ok = true;
    DateTime t = rtc.now();
    if (t.hour()   < 10) Serial.print('0'); Serial.print(t.hour());   Serial.print(':');
    if (t.minute() < 10) Serial.print('0'); Serial.print(t.minute()); Serial.print(':');
    if (t.second() < 10) Serial.print('0'); Serial.println(t.second());
  }

  Serial.println(F("====================================="));
  Serial.println(F("  Si4703 FM Radio — Tuning to 91.8  "));
  Serial.println(F("====================================="));
  Serial.println();

  // ── Step 1: Si4703 probe ──────────────────────────────
  Serial.println(F("[1] Checking Si4703..."));
  Wire.beginTransmission(SI4703_ADDR);
  uint8_t i2cErr = Wire.endTransmission();
  Serial.print(F("    I2C: ")); Serial.println(i2cErr == 0 ? F("OK") : F("FAIL"));

  // ── Step 2: Enable crystal oscillator ─────────────────
  Serial.println(F("[2] Starting crystal oscillator..."));
  readRegisters();
  regs[REG_TEST1] = 0x8100;  // XOSCEN + reserved bit 8 (required by AN332)
  writeRegisters();
  delay(500);
  Serial.println(F("    Done."));

  // ── Step 3: Power up the FM receiver ──────────────────
  Serial.println(F("[3] Powering up receiver..."));
  readRegisters();
  regs[REG_POWERCFG] = POWERCFG_DSMUTE | POWERCFG_DMUTE | POWERCFG_ENABLE;
  writeRegisters();
  delay(110);
  Serial.println(F("    Done."));

  // ── Step 4: Configure band, spacing, volume ───────────
  Serial.println(F("[4] Configuring band & volume..."));
  readRegisters();
  regs[REG_SYSCONFIG1] = SYSCONFIG1_DE;
  regs[REG_SYSCONFIG2] = (0x00 << 8) | (0b00 << 6) | (0b01 << 4) | 0x00;  // vol=0 at startup
  writeRegisters();
  Serial.println(F("    Band: 87.5-108 MHz, Spacing: 100 kHz, Volume: 0 (muted at boot)"));

  // ── Step 5: Tune to default station (91.8) ────────────────────────
  g_station = 1;
  Serial.print(F("[5] Tuning to ")); Serial.print(STATIONS[g_station]); Serial.println(F(" MHz..."));
  g_ok = tuneTo(STATIONS[g_station]);
  Serial.println(g_ok ? F("    Tuned.") : F("    Tune failed!"));

  printResult();

  // Prime the UI state machine and show the clock
  g_lastStationRaw = analogRead(STATION_POT);
  g_volEma         = analogRead(VOLUME_POT);  // prime EMA at actual pot position
  // Set volume to whatever the pot is currently at (not a jarring fixed value)
  uint8_t initVol = (uint8_t)computeVol(g_volEma);
  setVolume(initVol);
  Serial.print(F("    Volume set to pot position: ")); Serial.print(initVol); Serial.println(F("/15"));
  g_lastActivity   = millis();
  g_uiState        = UI_CLOCK;
  tft.fillScreen(ST77XX_BLACK);
}

// ── Curved text — parabola centred at x=160 (landscape midpoint) ──────────
static void curveDraw(const char* text, int size, uint16_t color, int baseY) {
  tft.setTextWrap(false);
  tft.setTextSize(size);
  int cw = 6 * size;
  int w  = (int)strlen(text) * cw;
  int x  = (320 - w) / 2;
  for (const char* p = text; *p; p++, x += cw) {
    float dx   = (float)(x + cw / 2) - 160.0f;
    int   yOff = (int)(0.0025f * dx * dx);
    tft.setTextColor(color, ST77XX_BLACK);
    tft.setCursor(x, baseY + yOff);
    tft.print(*p);
  }
}

// ── Tune the Si4703 to a frequency; updates g_freq/g_rssi/g_stereo ────────
bool tuneTo(float freqMHz) {
  uint16_t channel = (uint16_t)(((freqMHz - 87.5f) / 0.1f) + 0.5f);
  readRegisters();
  regs[REG_CHANNEL] = CHANNEL_TUNE | channel;
  writeRegisters();
  unsigned long start = millis();
  do {
    delay(15);
    readRegisters();
  } while (!(regs[REG_STATUSRSSI] & STATUS_STC) && millis() - start < 2000);
  bool ok = regs[REG_STATUSRSSI] & STATUS_STC;
  readRegisters();
  regs[REG_CHANNEL] &= ~CHANNEL_TUNE;
  writeRegisters();
  delay(10);
  readRegisters();
  uint16_t readChan = regs[REG_READCHAN] & 0x03FF;
  g_freq   = 87.5f + readChan * 0.1f;
  g_rssi   = regs[REG_STATUSRSSI] & 0xFF;
  g_stereo = regs[REG_STATUSRSSI] & STATUS_ST;
  return ok;
}

// ── Smoothed analog read for high-impedance (25K-250K guitar) pots ────────
// The ATmega ADC wants a source <=10K. These guitar pots are far higher, so the
// sample/hold cap can't settle — and it still holds charge from the previously
// muxed channel. Fix: select the channel with a throwaway read, pause to let the
// cap charge through the high resistance, THEN average several real samples.
int smoothRead(uint8_t pin) {
  analogRead(pin);            // dummy read — selects channel, starts cap charging
  delayMicroseconds(300);    // settle time for the high-Z source
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum >> 3);
}

// ── Big curved value (clears band first so variable-width text is clean) ──
void drawBigCurved(const char* s) {
  tft.fillRect(0, 50, 320, 85, ST77XX_BLACK);
  curveDraw(s, 5, LIGHTBLUE, 60);
}

// ── Map pot reading to a station zone (5 zones) with anti-bounce hysteresis ─
int stationZone(int raw, int cur) {
  const int zoneW  = 1024 / 5;   // ~204 counts per zone
  const int margin = 30;         // must clear boundary by this much to switch
  int z = raw / zoneW;
  if (z > 4) z = 4;
  if (z == cur) return cur;
  int boundary = (z > cur ? z : cur) * zoneW;
  if (abs(raw - boundary) >= margin) return z;
  return cur;                     // inside dead-band — hold current zone
}

// ── Screen: Clock (HH:MM) ─────────────────────────────────────────────
void screenClock(bool entering) {
  if (!g_rtc_ok) return;
  DateTime now = rtc.now();
  if (entering || now.minute() != g_shownMinute) {
    g_shownMinute = now.minute();
    char buf[6];
    sprintf(buf, "%02d:%02d", now.hour(), now.minute());
    drawBigCurved(buf);
  }
}

// ── Screen: Station selector (frequency) ─────────────────────────────
void screenStation(bool entering, int raw) {
  int z = stationZone(raw, g_station);
  if (entering || z != g_shownStation) {
    g_station      = z;
    g_shownStation = z;
    tuneTo(STATIONS[z]);
    int whole = (int)STATIONS[z];
    int frac  = (int)(STATIONS[z] * 10.0f + 0.5f) % 10;
    char buf[10];
    sprintf(buf, "%d.%dFM", whole, frac);
    drawBigCurved(buf);
    Serial.print(F("Station -> ")); Serial.print(STATIONS[z]); Serial.println(F(" MHz"));
  }
}

// ── Set Si4703 volume register (0-15) ────────────────────────────────────
void setVolume(uint8_t vol) {
  if (vol > 15) vol = 15;
  readRegisters();
  regs[REG_SYSCONFIG2] = (regs[REG_SYSCONFIG2] & 0xFFF0) | vol;
  writeRegisters();
  g_volume = vol;
}

// ── Compute volume level (0-15) from raw ADC; dead zones + log compensation ──
int computeVol(int raw) {
  if (raw < 72)   return 0;   // ~7% dead zone at bottom
  if (raw > 972)  return 15;
  int v = (int)(sqrtf((float)(raw - 72) / 900.0f) * 15.0f + 0.5f);
  return constrain(v, 0, 15);
}

// ── Screen: Volume ────────────────────────────────────────────────────────
void screenVolume(bool entering, int raw) {
  int vol = computeVol(raw);
  if (entering || vol != g_shownVol) {
    g_shownVol = vol;
    setVolume((uint8_t)vol);
    char buf[8];
    sprintf(buf, "%d/15", vol);
    drawBigCurved(buf);
    Serial.print(F("Volume -> ")); Serial.print(vol); Serial.println(F("/15"));
  }
}

void printResult() {
  Serial.println(F("====================================="));
  if (g_ok) {
    Serial.println(F("  TUNED — Audio active on jack       "));
    Serial.println(F("====================================="));
    Serial.print(F("  Frequency : ")); Serial.print(g_freq);  Serial.println(F(" MHz"));
    Serial.print(F("  RSSI      : ")); Serial.print(g_rssi);  Serial.println(F(" dBuV"));
    Serial.print(F("  Stereo    : ")); Serial.println(g_stereo ? F("Yes") : F("No (mono)"));
    Serial.println(F("  Volume    : 8 / 15 (50%)"));
  } else {
    Serial.println(F("  ERROR — Tune failed (check wiring) "));
  }
  Serial.println(F("====================================="));
  Serial.flush();
}

void loop() {
  unsigned long nowMs = millis();
  int stationRaw = smoothRead(STATION_POT);
  // EMA low-pass filter on the volume pot. smoothRead() already does a settling
  // dummy read so the high-Z pot value is clean before it feeds the filter.
  int volumeRaw = smoothRead(VOLUME_POT);
  g_volEma = (g_volEma * 7 + volumeRaw) >> 3;

  // ── Detect pot movement → switch screen + reset idle timer ──────────────
  if (abs(stationRaw - g_lastStationRaw) > POT_MOVE_THRESH) {
    g_lastStationRaw = stationRaw;
    g_uiState        = UI_STATION;
    g_lastActivity   = nowMs;
  }
  // Only switch to volume screen if the computed level has actually changed —
  // this prevents dead-zone jitter near raw=0 from triggering a spurious switch.
  if (computeVol(g_volEma) != g_volume) {
    g_uiState      = UI_VOLUME;
    g_lastActivity = nowMs;
  }

  // ── Idle timeout → return to clock ────────────────────────────────
  if (g_uiState != UI_CLOCK && nowMs - g_lastActivity > IDLE_TIMEOUT) {
    g_uiState = UI_CLOCK;
  }

  // ── On screen change: clear once and reset the "last shown" trackers ────
  bool entering = (g_uiState != g_prevState);
  if (entering) {
    tft.fillScreen(ST77XX_BLACK);
    g_shownMinute = g_shownStation = g_shownVol = -1;
    g_prevState = g_uiState;
  }

  switch (g_uiState) {
    case UI_CLOCK:   screenClock(entering);              break;
    case UI_STATION: screenStation(entering, stationRaw); break;
    case UI_VOLUME:  screenVolume(entering, g_volEma);   break;
  }

  delay(25);
}
