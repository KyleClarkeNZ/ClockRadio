// CJMCU-470 (Si4703) FM Clock Radio  +  ST7789 170x320 display  +  DS3231 RTC
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
#define LED_STEREO    8             // red LED: HIGH = stereo lock, LOW = mono/off
#define LED_BACKLIGHT 10            // LED filament strip (PWM): brightness tracks volume

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

// Auckland FM stations (MHz) — add or remove freely; the knob zones follow
const float STATIONS[] = { 88.6f, 90.2f, 91.8f, 94.2f, 95.0f, 99.0f };   // ascending = CCW to CW
const int   NUM_STATIONS = sizeof(STATIONS) / sizeof(STATIONS[0]);
int  g_station      = 0;   // index into STATIONS (set from the knob at boot)
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

// Si4703 reads always start at register 0x0A and wrap to 0x00.
// A short read (I2C timeout/glitch) leaves regs[] at its last good values —
// otherwise Wire.read()'s -1 would fill it with 0xFFFF, and the next
// writeRegisters() would send that garbage to the chip (power-down, seek...).
bool readRegisters() {
  if (Wire.requestFrom((uint8_t)SI4703_ADDR, (uint8_t)32) != 32) {
    while (Wire.available()) Wire.read();   // discard the partial read
    return false;
  }
  for (int i = 0x0A; i <= 0x0F; i++)
    regs[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
  for (int i = 0x00; i <= 0x09; i++)
    regs[i] = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
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
  Serial.println(F("  Si4703 FM Clock Radio             "));
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
  regs[REG_TEST1] = TEST1_XOSCEN | 0x0100;  // + reserved bit 8 (required by AN332)
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

  // ── Step 5: Tune to the station the knob is pointing at ─────────────
  // smoothRead() (not analogRead) so the high-Z pots settle before priming.
  g_lastStationRaw = smoothRead(STATION_POT);
  g_volEma         = smoothRead(VOLUME_POT);
  g_station = stationZoneRaw(g_lastStationRaw);
  Serial.print(F("[5] Tuning to ")); Serial.print(STATIONS[g_station]); Serial.println(F(" MHz..."));
  g_ok = tuneTo(STATIONS[g_station]);
  Serial.println(g_ok ? F("    Tuned.") : F("    Tune failed!"));

  printResult();

#ifdef BAND_SCAN
  // Debug: tune every 100 kHz channel and print RSSI, then return to the station
  while (!Serial && millis() < 15000);   // wait for the serial monitor to attach
  Serial.println(F("SCAN freq rssi stereo"));
  for (int ch = 0; ch <= 205; ch++) {
    tuneTo(87.5f + ch * 0.1f);
    Serial.print(F("SCAN ")); Serial.print(g_freq, 1); Serial.print(' ');
    Serial.print(g_rssi); Serial.print(' '); Serial.println(g_stereo ? 1 : 0);
  }
  Serial.println(F("SCAN done"));
  g_ok = tuneTo(STATIONS[g_station]);
#endif

  // Configure LED pins before setVolume() so the first call sets them correctly
  pinMode(LED_STEREO,    OUTPUT);
  pinMode(LED_BACKLIGHT, OUTPUT);
  // Set volume to whatever the pot is currently at (not a jarring fixed value);
  // setVolume() will drive both LEDs to the right state immediately.
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
// Also: the pots' wipers go open-circuit at the end stops (and at full CW when
// the knob is pressed), so the pin floats and reads whatever charge the cap last
// held. To detect that, the pin is read twice: once after pre-charging the cap
// from the internal 0V channel, once from the 1.1V bandgap. A connected wiper
// drives both to the same value; a floating pin follows the pre-charge, so the
// reads disagree and the last good value is returned instead.
#define ADC_MUX_BANDGAP 0x1E   // 1.1V reference
#define ADC_MUX_GND     0x1F   // 0V
const int FLOAT_THRESH = 40;   // counts of disagreement that mean "floating"
int g_floatDiff = 0;           // last disagreement seen (for POT_DEBUG)

static void adcPrecharge(uint8_t mux) {
  ADMUX  = _BV(REFS0) | mux;           // AVcc ref + internal channel
  ADCSRB &= ~_BV(MUX5);
  for (uint8_t i = 0; i < 2; i++) {    // 2 conversions: bandgap needs to settle
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
  }
}

static int settledRead(uint8_t pin, uint8_t prechargeMux) {
  adcPrecharge(prechargeMux);
  analogRead(pin);            // dummy read — selects channel, starts cap charging
  delayMicroseconds(300);    // settle time for the high-Z source
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(pin);
    delayMicroseconds(50);
  }
  return (int)(sum >> 3);
}

// Some floating reads still agree by chance (mains hum drives the open pin), so
// two more guards: after any floating read the pot is ignored for FLOAT_LOCKOUT
// ms, and a jump bigger than MAX_STEP is only accepted once JUMP_CONFIRM reads
// in a row agree with each other — a real turn moves smoothly, a lifted wiper
// jumps straight from ~1017 to a few hundred.
const unsigned long FLOAT_LOCKOUT = 250;
const int MAX_STEP     = 80;
const uint8_t JUMP_CONFIRM = 3;

int smoothRead(uint8_t pin) {
  // Per-pot state, indexed by pin - A0 (A0 / A1)
  static int           lastGood[2]  = { -1, -1 };   // -1 = not primed yet
  static int           prevSample[2];
  static uint8_t       runCount[2];
  static unsigned long floatUntil[2];
  int idx = (pin - A0) & 1;

  int lo = settledRead(pin, ADC_MUX_GND);
  int hi = settledRead(pin, ADC_MUX_BANDGAP);
  if (pin == VOLUME_POT) g_floatDiff = hi - lo;
  unsigned long now = millis();
  if (abs(hi - lo) > FLOAT_THRESH) {                 // wiper open — hold
    floatUntil[idx] = now + FLOAT_LOCKOUT;
    runCount[idx]   = 0;
    return max(lastGood[idx], 0);
  }
  if ((long)(now - floatUntil[idx]) < 0) return max(lastGood[idx], 0);

  int v = (lo + hi) >> 1;
  if (lastGood[idx] < 0 || abs(v - lastGood[idx]) <= MAX_STEP) {
    lastGood[idx] = v;
    runCount[idx] = 0;
  } else if (abs(v - prevSample[idx]) <= MAX_STEP) {
    if (++runCount[idx] >= JUMP_CONFIRM - 1) {      // this read + the run before it
      lastGood[idx] = v;
      runCount[idx] = 0;
    }
  } else {
    runCount[idx] = 0;
  }
  prevSample[idx] = v;
  return lastGood[idx];
}

// ── Big curved value drawn in place — no per-update clear, no flash ──────
// curveDraw() uses an opaque black background on every glyph cell, so each
// character overwrites its own pixels in place. No fillRect needed here;
// the text band is cleared once in loop() on screen transitions (entering).
void drawBigCurved(const char* s) {
  curveDraw(s, 5, LIGHTBLUE, 60);
}

// ── Map pot reading to a station zone (one per station) with log compensation ─
// A1 is a log/audio taper pot; sqrt() maps its response to a linear 0.0-1.0
// position before zoning, so every station gets equal physical rotation.
// Hysteresis: must move 0.2 of a zone past the boundary to switch.
static float stationPos(int raw) {
  return sqrtf((float)constrain(raw, 0, 1023) / 1023.0f) * NUM_STATIONS;  // 0.0-NUM_STATIONS
}

// Zone with no hysteresis — used at boot when there is no current station
int stationZoneRaw(int raw) {
  int z = (int)stationPos(raw);
  return z > NUM_STATIONS - 1 ? NUM_STATIONS - 1 : z;
}

int stationZone(int raw, int cur) {
  float pos = stationPos(raw);
  int z = stationZoneRaw(raw);
  if (z == cur) return cur;
  if (abs(z - cur) > 1) return z;               // big jump — clearly intentional
  // Only commit to new zone if we're 0.2 zone-widths clear of the boundary
  float boundary = (float)(z > cur ? z : cur);  // the shared edge between zones
  if (fabsf(pos - boundary) >= 0.2f) return z;
  return cur;
}

// ── Screen: Clock (12-hour HH:MM, zero-padded) ─────────────────────────
void screenClock(bool entering) {
  if (!g_rtc_ok) return;
  DateTime now = rtc.now();
  int min = now.minute();
  int h12 = now.hour() % 12;
  if (h12 == 0) h12 = 12;

  if (entering || min != g_shownMinute) {
    g_shownMinute = min;
    char buf[6];
    sprintf(buf, "%02d:%02d", h12, min);
    drawBigCurved(buf);
  }
}

// ── Screen: Station selector (frequency) — display only; loop() tunes ────
void screenStation(bool entering) {
  int z = g_station;
  if (entering || z != g_shownStation) {
    g_shownStation = z;
    int whole = (int)STATIONS[z];
    int frac  = (int)(STATIONS[z] * 10.0f + 0.5f) % 10;
    char buf[10];
    sprintf(buf, "%d.%dFM", whole, frac);
    drawBigCurved(buf);
  }
}

// ── Set Si4703 volume register (0-15) ────────────────────────────────────
void setVolume(uint8_t vol) {
  if (vol > 15) vol = 15;
  readRegisters();
  regs[REG_SYSCONFIG2] = (regs[REG_SYSCONFIG2] & 0xFFF0) | vol;
  writeRegisters();
  g_volume = vol;
  analogWrite(LED_BACKLIGHT, vol * 17);            // 0-15 -> 0-255 PWM
  digitalWrite(LED_STEREO, vol > 0 ? HIGH : LOW);  // on when audio active
}

// ── Compute volume level (0-15) from raw ADC: linear, 16 equal-width bands ──
// Ends measured on the real pot: CCW end floats 0-100 (mains hum), CW ~1016-1020.
const int VOL_RAW_MIN = 110;   // at or below -> 0 (mute); floating CCW end hums 0-100
const int VOL_RAW_MAX = 990;   // at or above -> 15
int computeVol(int raw) {
  if (raw <= VOL_RAW_MIN) return 0;
  if (raw >= VOL_RAW_MAX) return 15;
  return 1 + (long)(raw - VOL_RAW_MIN) * 14 / (VOL_RAW_MAX - VOL_RAW_MIN);
}

// ── Volume level with hysteresis: the pot must move VOL_HYST counts past a
// step edge before the level changes, so noise near an edge can't flicker it.
const int VOL_HYST = 8;
int volumeLevel(int raw, int cur) {
  int v = computeVol(raw);
  if (v > cur && computeVol(raw - VOL_HYST) > cur) return v;
  if (v < cur && computeVol(raw + VOL_HYST) < cur) return v;
  return cur;
}

// ── Screen: Volume — display only; loop() applies the volume ─────────────
void screenVolume(bool entering) {
  int vol = g_volume;
  if (entering || vol != g_shownVol) {
    g_shownVol = vol;
    char buf[8];
    sprintf(buf, "%2d/15", vol);   // always 5 chars — fixed width avoids yOff mismatch
    drawBigCurved(buf);
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
  g_volEma = (g_volEma * 7 + volumeRaw + 4) >> 3;   // +4 rounds to nearest

#ifdef POT_DEBUG
  static unsigned long lastDbg = 0;
  if (nowMs - lastDbg > 200) {
    lastDbg = nowMs;
    Serial.print(F("DBG volRaw=")); Serial.print(volumeRaw);
    Serial.print(F(" ema="));       Serial.print(g_volEma);
    Serial.print(F(" lvl="));       Serial.print(g_volume);
    Serial.print(F(" diff="));      Serial.println(g_floatDiff);
  }
#endif

  // ── Apply knob changes regardless of which screen is showing ────────────
  // Tuning/volume live here (not in the screen functions) so a change is never
  // lost when both knobs move at once, and redrawing a screen never re-tunes.
  int z = stationZone(stationRaw, g_station);
  if (z != g_station) {
    g_station = z;
    tuneTo(STATIONS[z]);
    Serial.print(F("Station -> ")); Serial.print(STATIONS[z]); Serial.println(F(" MHz"));
    g_uiState      = UI_STATION;
    nowMs          = millis();   // tuneTo() can block — refresh so the idle check
    g_lastActivity = nowMs;      // below can't see lastActivity > nowMs and wrap
  }
  int vol = volumeLevel(g_volEma, g_volume);
  if (vol != g_volume) {
    setVolume((uint8_t)vol);
    Serial.print(F("Volume -> ")); Serial.print(vol); Serial.println(F("/15"));
    g_uiState      = UI_VOLUME;
    g_lastActivity = nowMs;
  }

  // ── Knob touched without changing station → show/hold the station screen ─
  if (abs(stationRaw - g_lastStationRaw) > POT_MOVE_THRESH) {
    g_lastStationRaw = stationRaw;
    if (g_uiState != UI_VOLUME) g_uiState = UI_STATION;
    g_lastActivity = nowMs;
  }

  // ── Idle timeout → return to clock ────────────────────────────────
  if (g_uiState != UI_CLOCK && nowMs - g_lastActivity > IDLE_TIMEOUT) {
    g_uiState = UI_CLOCK;
  }

  // ── On screen change: clear the text band once, then force a full redraw. ──
  // Clearing only on entering (not per-update) means minute ticks / volume
  // steps / station changes never blank anything — they overwrite in place.
  // The one-time band clear on entering removes vertical top-of-cell remnants
  // that linger when an old char at a given x started higher (smaller yOff)
  // than the new char at the same x range.
  bool entering = (g_uiState != g_prevState);
  if (entering) {
    tft.fillRect(0, 50, 320, 85, ST77XX_BLACK);   // clear text band only
    g_shownMinute = g_shownStation = g_shownVol = -1;
    g_prevState = g_uiState;
  }

  switch (g_uiState) {
    case UI_CLOCK:   screenClock(entering);              break;
    case UI_STATION: screenStation(entering); break;
    case UI_VOLUME:  screenVolume(entering);  break;
  }

  delay(25);
}
