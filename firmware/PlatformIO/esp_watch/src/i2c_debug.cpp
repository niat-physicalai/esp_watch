/*
  I2C Bus Diagnostic - ESP32 + MPU6050 + MAX30102 + SSD1306
  =========================================================
  Finds out WHY the MPU disappears when the MAX is connected.

  Wiring: all three modules on SDA=21, SCL=22, 3V3, GND.

  OPTIONAL but very useful (lets the sketch measure the bus voltage
  without a multimeter): run two extra jumper wires
        SDA (21) --> GPIO34
        SCL (22) --> GPIO35
  GPIO34/35 are input-only ADC pins and will not disturb the bus.
  Set MEASURE_LEVELS to 0 if you don't want to add these wires.

  Serial monitor: 115200 baud.

  PlatformIO: copy this file to src/main.cpp (the Arduino.h include below is
  what the Arduino IDE would otherwise add for you).

  What it does:
    1. Measures the idle voltage on SDA and SCL
    2. Scans the bus at 100 kHz and at 400 kHz
    3. Probes each chip by reading its ID register
    4. Hammers each chip with 200 reads and reports the error rate
    5. Repeats every 4 s, so you can plug/unplug modules and watch live
*/

#include <Arduino.h>     // required when built as main.cpp under PlatformIO
#include <Wire.h>

#define PIN_SDA 21
#define PIN_SCL 22

#define MEASURE_LEVELS 1     // set 0 if you skip the GPIO34/35 jumpers
#define ADC_SDA 34
#define ADC_SCL 35

#define ADDR_OLED 0x3C
#define ADDR_MAX  0x57
#define ADDR_MPU  0x68
#define ADDR_MPU2 0x69       // MPU with AD0 pulled high

#define WIGGLE_TEST 1        // 1 = run the live wire-wiggle test each round

uint32_t busClock = 100000;

// ---------- low-level helpers ----------
bool busAlive = false;      // true once any device has answered

bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Read n bytes. repeatedStart=false sends a STOP between the write and the
// read, which is what the MPU6050 / SSD1306 libraries actually do.
bool readRegs(uint8_t addr, uint8_t reg, uint8_t* out, uint8_t n, bool repeatedStart) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(!repeatedStart) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) out[i] = Wire.read();
  return true;
}

// Returns true on success, value in *out
bool readReg(uint8_t addr, uint8_t reg, uint8_t* out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;    // repeated start
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  *out = Wire.read();
  return true;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// ---------- 1. bus voltage ----------
void checkLevels() {
#if MEASURE_LEVELS
  // Let the bus idle high, then measure
  delay(5);
  int sda = analogReadMilliVolts(ADC_SDA);
  int scl = analogReadMilliVolts(ADC_SCL);
  Serial.printf("  Idle bus voltage : SDA %d mV | SCL %d mV\n", sda, scl);
  int lo = min(sda, scl);
  if (lo < 500 && busAlive) {
    Serial.println("  (Those readings are meaningless - GPIO34/35 are not");
    Serial.println("   wired to the bus, so they float near 0 V. Devices are");
    Serial.println("   answering normally, so the real bus level is fine.)");
  } else if (lo < 2600) {
    Serial.println("  >> PROBLEM: idle level is well below 3.3 V.");
    Serial.println("     Something is pulling the bus toward a lower rail.");
    Serial.println("     The usual cause is the MAX30102 module: its pull-up");
    Serial.println("     resistors go to its internal 1.8 V rail, not 3.3 V.");
  } else if (lo < 3000) {
    Serial.println("  >> MARGINAL: idle level is low. Expect random failures.");
  } else {
    Serial.println("  Levels look healthy (>3.0 V).");
  }
#else
  // Crude fallback: read the pins as digital inputs
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delayMicroseconds(200);
  int sda = digitalRead(PIN_SDA), scl = digitalRead(PIN_SCL);
  Serial.printf("  Idle digital level: SDA %s | SCL %s\n",
                sda ? "HIGH" : "LOW", scl ? "HIGH" : "LOW");
  if (!sda || !scl)
    Serial.println("  >> PROBLEM: a line reads LOW while idle. Either a device "
                   "is holding it down, or the pull-up voltage is too low for "
                   "the ESP32 to see as HIGH (needs ~2.5 V).");
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(busClock);
#endif
}

// ---------- 2. scan ----------
int scan(uint32_t clk) {
  Wire.setClock(clk);
  Serial.printf("  Scan @ %lu kHz: ", (unsigned long)(clk / 1000));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (ping(a)) {
      Serial.printf("0x%02X ", a);
      n++;
    }
  }
  if (!n) Serial.print("(nothing)");
  Serial.println();
  Wire.setClock(busClock);
  return n;
}

// ---------- 3. identify chips ----------
void identify() {
  uint8_t v;

  // --- MPU6050: WHO_AM_I = 0x75, usually 0x68 (clones return 0x70, 0x72, 0x98) ---
  uint8_t mpuAddr = 0;
  if (ping(ADDR_MPU))       mpuAddr = ADDR_MPU;
  else if (ping(ADDR_MPU2)) mpuAddr = ADDR_MPU2;

  if (!mpuAddr) {
    Serial.println("  MPU6050  : NOT FOUND at 0x68 or 0x69");
  } else {
    Serial.printf("  MPU6050  : responds at 0x%02X", mpuAddr);
    if (readReg(mpuAddr, 0x75, &v)) Serial.printf(", WHO_AM_I = 0x%02X", v);
    else                            Serial.print(", but register read FAILED");
    // Wake it and read one accel byte to prove data flows
    writeReg(mpuAddr, 0x6B, 0x00);
    if (readReg(mpuAddr, 0x3B, &v)) Serial.printf(", accel_xh = 0x%02X", v);
    Serial.println();
    if (mpuAddr == ADDR_MPU2)
      Serial.println("     >> AD0 is HIGH. Tie the MPU's AD0 pin to GND for 0x68.");
  }

  // --- MAX30102: PART_ID register 0xFF should read 0x15 ---
  if (!ping(ADDR_MAX)) {
    Serial.println("  MAX30102 : NOT FOUND at 0x57");
  } else {
    Serial.print("  MAX30102 : responds at 0x57");
    if (readReg(ADDR_MAX, 0xFF, &v)) {
      Serial.printf(", PART_ID = 0x%02X %s", v, (v == 0x15) ? "(correct)" : "(WRONG - bad read)");
    } else {
      Serial.print(", but register read FAILED");
    }
    Serial.println();
  }

  // --- OLED: only ACKs, has no readable ID ---
  Serial.printf("  SSD1306  : %s at 0x3C%s\n",
                ping(ADDR_OLED) ? "responds" : "NOT FOUND",
                ping(ADDR_OLED) ? " (write-only: ACK does not prove clean data)" : "");
}

// ---------- 4. stress test ----------
void stressEx(const char* label, uint8_t addr, uint8_t reg,
              bool repeatedStart, int gapMs) {
  if (!ping(addr)) return;
  const int TRIES = 200;
  int nack = 0, mismatch = 0;
  uint8_t v, first = 0;
  bool got = false;
  unsigned long t0 = millis();
  for (int i = 0; i < TRIES; i++) {
    if (!readRegs(addr, reg, &v, 1, repeatedStart)) nack++;
    else if (!got) { first = v; got = true; }
    else if (v != first) mismatch++;
    if (gapMs) delay(gapMs);
  }
  int bad = nack + mismatch;
  Serial.printf("    %-34s %3d/%d bad (no-answer %d, wrong value %d) %lums %s\n",
                label, bad, TRIES, nack, mismatch, millis() - t0,
                bad == 0 ? "OK" : "<-- fails");
}

// Closest thing to what the watch firmware really does: 6 accel bytes,
// STOP between write and read, 50 Hz, for 2 seconds.
void accelRealWorld(uint8_t addr) {
  if (!ping(addr)) return;
  uint8_t b[6];
  int bad = 0, n = 100;
  long lastX = 0;
  int frozen = 0;
  for (int i = 0; i < n; i++) {
    if (!readRegs(addr, 0x3B, b, 6, false)) bad++;
    else {
      long x = (int16_t)((b[0] << 8) | b[1]);
      if (i && x == lastX) frozen++;
      lastX = x;
    }
    delay(20);
  }
  Serial.printf("    %-34s %3d/%d bad, %d identical samples\n",
                "accel burst 6B, STOP, 50 Hz", bad, n, frozen);
  Serial.printf("    last accel X = %ld (tilt the board: this should change)\n", lastX);
}

void stress(const char* name, uint8_t addr, uint8_t reg) {
  if (!ping(addr)) return;
  const int TRIES = 200;
  int fails = 0;
  uint8_t v, first = 0;
  bool got = false;
  for (int i = 0; i < TRIES; i++) {
    if (!readReg(addr, reg, &v)) fails++;
    else if (!got) { first = v; got = true; }
    else if (reg == 0x75 || reg == 0xFF) {
      if (v != first) fails++;              // ID registers must never change
    }
  }
  Serial.printf("  %-9s: %d/%d reads failed or were corrupt (%.1f%%)%s\n",
                name, fails, TRIES, 100.0 * fails / TRIES,
                fails == 0 ? "" : "  <-- unreliable");
}

// ---------- setup / loop ----------
// Continuous test of one device: wiggle the wires and watch the bar change
void wiggleTest(uint8_t addr, uint8_t reg, const char* name) {
  if (!ping(addr)) {
    Serial.printf("  %s not answering - skipping wiggle test\n", name);
    return;
  }
  Serial.printf("\n  WIGGLE TEST on %s - move its wires now (6 s):\n", name);
  for (int sec = 0; sec < 12; sec++) {
    int ok = 0;
    uint8_t v;
    for (int i = 0; i < 50; i++) if (readReg(addr, reg, &v)) ok++;
    Serial.print("   [");
    for (int i = 0; i < 25; i++) Serial.print(i < ok / 2 ? '#' : '.');
    Serial.printf("] %d%%\n", ok * 2);
    delay(200);
  }
  Serial.println("  If the bar dips while you touch a wire, that wire or its");
  Serial.println("  solder joint is the fault. If it is steadily bad without");
  Serial.println("  touching anything, suspect AD0 or the module itself.\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);
#if MEASURE_LEVELS
  analogReadResolution(12);
  pinMode(ADC_SDA, INPUT);
  pinMode(ADC_SCL, INPUT);
#endif
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(busClock);
  Serial.println("\n\n========== I2C BUS DIAGNOSTIC ==========");
  Serial.println("Unplug/replug modules while this runs to compare.");
}

void loop() {
  Serial.println("\n---------------------------------------");
  int n100 = scan(100000);
  busAlive = (n100 > 0);
  checkLevels();
  int n400 = scan(400000);
  if (n400 < n100)
    Serial.println("  >> Devices drop out at 400 kHz: wiring is too marginal "
                   "for the faster speed (long wires, weak pull-ups).");
  identify();
  Serial.println("  Reliability:");
  Serial.println("  MPU6050 - four access styles, to find what it dislikes:");
  stressEx("repeated start, no gap",  ADDR_MPU, 0x75, true,  0);
  stressEx("STOP between, no gap",    ADDR_MPU, 0x75, false, 0);
  stressEx("repeated start, 2 ms gap",ADDR_MPU, 0x75, true,  2);
  stressEx("STOP between, 2 ms gap",  ADDR_MPU, 0x75, false, 2);
  accelRealWorld(ADDR_MPU);
  Serial.println("  MAX30102:");
  stressEx("repeated start, no gap",  ADDR_MAX, 0xFF, true,  0);
  stressEx("STOP between, no gap",    ADDR_MAX, 0xFF, false, 0);

  if (WIGGLE_TEST) {
    wiggleTest(ADDR_MPU, 0x75, "MPU6050");
  }
  delay(4000);
}
