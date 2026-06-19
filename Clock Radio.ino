// CJMCU-470 (Si4703) FM Radio - Tune to 91.8 MHz
//
// Wiring:
//   CJMCU-470 VCC  -> Arduino 3.3V
//   CJMCU-470 GND  -> Arduino GND
//   CJMCU-470 SDA  -> Arduino SDA (pin 2 on Micro)
//   CJMCU-470 SCL  -> Arduino SCL (pin 3 on Micro)
//   CJMCU-470 RST  -> Arduino pin 5
//
// Audio output is on the CJMCU-470 onboard 3.5mm headphone jack.

#include <Wire.h>

#define SI4703_ADDR   0x10
#define SI4703_RST    5

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

// Results shared between setup() and loop()
float    g_freq   = 0;
uint8_t  g_rssi   = 0;
bool     g_stereo = false;
bool     g_ok     = false;

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
  // Don't block waiting for serial — run regardless, results printed in loop() too
  delay(200);

  Serial.println(F("====================================="));
  Serial.println(F("  Si4703 FM Radio — Tuning to 91.8  "));
  Serial.println(F("====================================="));
  Serial.println();

  // ── Step 1: Hardware reset ─────────────────────────────
  Serial.println(F("[1] Resetting Si4703..."));
  pinMode(SI4703_RST, OUTPUT);
  digitalWrite(SI4703_RST, LOW);
  delay(10);
  digitalWrite(SI4703_RST, HIGH);
  delay(10);
  Wire.begin();
  Serial.println(F("    Done."));

  // ── Step 2: Enable crystal oscillator ─────────────────
  Serial.println(F("[2] Starting crystal oscillator..."));
  readRegisters();
  regs[REG_TEST1] = 0x8100;  // XOSCEN + reserved bit 8 (required by AN332)
  writeRegisters();
  delay(500);  // Datasheet: min 500 ms for crystal stabilisation
  Serial.println(F("    Done."));

  // ── Step 3: Power up the FM receiver ──────────────────
  Serial.println(F("[3] Powering up receiver..."));
  readRegisters();
  regs[REG_POWERCFG] = POWERCFG_DSMUTE | POWERCFG_DMUTE | POWERCFG_ENABLE;
  writeRegisters();
  delay(110);  // Datasheet: 110 ms powerup time
  Serial.println(F("    Done."));

  // ── Step 4: Configure band, spacing, volume ───────────
  Serial.println(F("[4] Configuring band & volume..."));
  readRegisters();
  regs[REG_SYSCONFIG1] = SYSCONFIG1_DE;        // 50µs de-emphasis (Europe)
  // SEEKTH=0x00, BAND=00 (87.5–108 MHz), SPACE=01 (100 kHz), VOLUME=1111 (max)
  regs[REG_SYSCONFIG2] = (0x00 << 8) | (0b00 << 6) | (0b01 << 4) | 0x0F;
  writeRegisters();
  Serial.println(F("    Band: 87.5-108 MHz, Spacing: 100 kHz, Volume: 15/15"));

  // ── Step 5: Tune to 91.8 MHz ──────────────────────────
  uint16_t channel = (uint16_t)(((TARGET_FREQ - 87.5f) / 0.1f) + 0.5f);  // = 43
  float    actualFreq = 87.5f + channel * 0.1f;

  Serial.print(F("[5] Tuning to "));
  Serial.print(actualFreq);
  Serial.print(F(" MHz (channel "));
  Serial.print(channel);
  Serial.println(F(")..."));

  readRegisters();
  regs[REG_CHANNEL] = CHANNEL_TUNE | channel;
  writeRegisters();

  // Wait for STC (Seek/Tune Complete) flag
  unsigned long start = millis();
  do {
    delay(20);
    readRegisters();
  } while (!(regs[REG_STATUSRSSI] & STATUS_STC) && millis() - start < 4000);

  if (!(regs[REG_STATUSRSSI] & STATUS_STC)) {
    Serial.println(F("    ERROR: Tune timed out!"));
    return;
  }

  // Clear TUNE bit
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
}

void printResult() {
  Serial.println(F("====================================="));
  if (g_ok) {
    Serial.println(F("  TUNED — Audio active on jack       "));
    Serial.println(F("====================================="));
    Serial.print(F("  Frequency : ")); Serial.print(g_freq);  Serial.println(F(" MHz"));
    Serial.print(F("  RSSI      : ")); Serial.print(g_rssi);  Serial.println(F(" dBuV"));
    Serial.print(F("  Stereo    : ")); Serial.println(g_stereo ? F("Yes") : F("No (mono)"));
    Serial.println(F("  Volume    : 15 / 15 (max)"));
  } else {
    Serial.println(F("  ERROR — Tune failed (check wiring) "));
  }
  Serial.println(F("====================================="));
}

void loop() {
  // Re-print every 5 s so a late serial connection still sees the result
  delay(5000);
  printResult();
}
