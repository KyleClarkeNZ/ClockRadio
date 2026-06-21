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

uint16_t regs[16];

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

RTC_DS3231 rtc;
bool       g_rtc_ok = false;

// Results shared between setup() and loop()
float    g_freq   = 0;
uint8_t  g_rssi   = 0;
bool     g_stereo = false;
bool     g_ok     = false;

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
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(37, 8);
  tft.print(F("FM RADIO"));
  tft.drawFastHLine(0, 30, 170, ST77XX_CYAN);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(8, 50);
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
  regs[REG_SYSCONFIG2] = (0x00 << 8) | (0b00 << 6) | (0b01 << 4) | 0x08;
  writeRegisters();
  Serial.println(F("    Band: 87.5-108 MHz, Spacing: 100 kHz, Volume: 8/15 (50%)"));

  // ── Step 5: Tune to 91.8 MHz ──────────────────────────
  uint16_t channel = (uint16_t)(((TARGET_FREQ - 87.5f) / 0.1f) + 0.5f);
  float    actualFreq = 87.5f + channel * 0.1f;
  Serial.print(F("[5] Tuning to ")); Serial.print(actualFreq);
  Serial.print(F(" MHz (channel ")); Serial.print(channel); Serial.println(F(")..."));

  readRegisters();
  regs[REG_CHANNEL] = CHANNEL_TUNE | channel;
  writeRegisters();

  unsigned long start = millis();
  do {
    delay(20);
    readRegisters();
  } while (!(regs[REG_STATUSRSSI] & STATUS_STC) && millis() - start < 4000);

  if (!(regs[REG_STATUSRSSI] & STATUS_STC)) {
    Serial.println(F("    ERROR: Tune timed out!"));
    drawStaticLayout();  // shows error screen
    return;
  }

  readRegisters();
  regs[REG_CHANNEL] &= ~CHANNEL_TUNE;
  writeRegisters();
  delay(10);

  // ── Step 6: Read back and report ──────────────────────
  readRegisters();
  uint16_t readChan = regs[REG_READCHAN] & 0x03FF;
  g_freq   = 87.5f + readChan * 0.1f;
  g_rssi   = regs[REG_STATUSRSSI] & 0xFF;
  g_stereo = regs[REG_STATUSRSSI] & STATUS_ST;
  g_ok     = true;

  printResult();
  drawStaticLayout();  // full draw once — loop() only updates dynamic regions
}

// ── Helper: print 2-digit zero-padded number to display ──────────────────
static void tftPad2(uint8_t v) {
  if (v < 10) tft.print('0');
  tft.print(v);
}

// ── Helper: month name ────────────────────────────────────────────────────
static void printMonthName(uint8_t m) {
  switch (m) {
    case  1: tft.print(F("Jan")); break;  case  2: tft.print(F("Feb")); break;
    case  3: tft.print(F("Mar")); break;  case  4: tft.print(F("Apr")); break;
    case  5: tft.print(F("May")); break;  case  6: tft.print(F("Jun")); break;
    case  7: tft.print(F("Jul")); break;  case  8: tft.print(F("Aug")); break;
    case  9: tft.print(F("Sep")); break;  case 10: tft.print(F("Oct")); break;
    case 11: tft.print(F("Nov")); break;  case 12: tft.print(F("Dec")); break;
  }
}

// ── Helper: day-of-week name ──────────────────────────────────────────────
static void printDayName(uint8_t d) {
  switch (d) {
    case 0: tft.print(F("Sunday"));    break;  case 1: tft.print(F("Monday"));    break;
    case 2: tft.print(F("Tuesday"));   break;  case 3: tft.print(F("Wednesday")); break;
    case 4: tft.print(F("Thursday"));  break;  case 5: tft.print(F("Friday"));    break;
    case 6: tft.print(F("Saturday"));  break;
  }
}

// ── RSSI region (y=92..133) — call to refresh without full screen wipe ────
void drawRSSI() {
  uint16_t col = (g_rssi >= 30) ? (uint16_t)ST77XX_GREEN
               : (g_rssi >= 15) ? (uint16_t)ST77XX_YELLOW
               :                  (uint16_t)ST77XX_RED;
  // Text with background colour — auto-erases previous value of any width
  tft.setTextColor(col, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(1, 92);
  tft.print(F("RSSI: "));
  tft.print(g_rssi);
  tft.print(F(" dBuV   "));  // trailing spaces clear leftover digits
  // Bar: clear full width then draw active portion
  uint8_t barW = (uint8_t)constrain(map(g_rssi, 0, 50, 0, 150), 0, 150);
  tft.fillRect(10, 116, 150, 10, ST77XX_BLACK);
  if (barW > 0) tft.fillRect(10, 116, barW, 10, col);
  tft.drawRect(10, 116, 150, 10, ST77XX_WHITE);
}

// ── Clock region (y=220..292) — call every second ─────────────────────────
void drawTime() {
  if (!g_rtc_ok) return;
  DateTime now = rtc.now();
  // Clear only the clock zone — rest of screen is untouched
  tft.fillRect(0, 220, 170, 72, ST77XX_BLACK);
  tft.setTextWrap(false);
  // Time: "HH:MM:SS" size 3 = 8×18=144px, centred x=13
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(13, 220);
  tftPad2(now.hour()); tft.print(':');
  tftPad2(now.minute()); tft.print(':');
  tftPad2(now.second());
  // Date: "DD Mon YYYY" size 2 = 11×12=132px, centred x=19
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(19, 248);
  tftPad2(now.day()); tft.print(' ');
  printMonthName(now.month()); tft.print(' ');
  tft.print(now.year());
  // Day name size 2, left-padded
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(8, 268);
  printDayName(now.dayOfTheWeek());
}

// ── Full static layout — called ONCE after tune ───────────────────────────
void drawStaticLayout() {
  tft.fillScreen(ST77XX_BLACK);  // only full clear ever done after boot
  tft.setTextWrap(false);

  // Title
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(37, 8);
  tft.print(F("FM RADIO"));
  tft.drawFastHLine(0, 30, 170, ST77XX_CYAN);

  if (!g_ok) {
    tft.setTextColor(ST77XX_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 60);
    tft.print(F("TUNE ERROR"));
    tft.setTextSize(1);
    tft.setCursor(8, 90);
    tft.print(F("Check wiring"));
    return;
  }

  // Frequency — static after tune
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(4, 42);
  tft.print(g_freq, 2);
  tft.print(F(" MHz"));

  tft.drawFastHLine(0, 82, 170, ST77XX_CYAN);
  drawRSSI();  // dynamic region — initial draw
  tft.drawFastHLine(0, 134, 170, ST77XX_CYAN);

  // Stereo / Mono — static after tune
  tft.setTextSize(3);
  if (g_stereo) {
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(31, 145);
    tft.print(F("STEREO"));
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(49, 145);
    tft.print(F("MONO"));
  }

  tft.drawFastHLine(0, 182, 170, ST77XX_CYAN);

  // Volume — static
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(19, 192);
  tft.print(F("Vol: 8 / 15"));

  tft.drawFastHLine(0, 212, 170, ST77XX_CYAN);
  drawTime();  // dynamic region — initial draw
}

// ── updateDynamic — called from loop(), no full-screen wipe ───────────────
void updateDynamic() {
  // Re-read Si4703 status for current RSSI
  readRegisters();
  uint8_t newRssi = regs[REG_STATUSRSSI] & 0xFF;
  if (newRssi != g_rssi) {
    g_rssi = newRssi;
    drawRSSI();
  }
  drawTime();
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
  delay(1000);      // 1 s — drives the clock and RSSI refresh
  updateDynamic();  // updates only RSSI bar and clock region; no fillScreen
}
