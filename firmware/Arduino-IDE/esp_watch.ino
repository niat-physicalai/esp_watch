/*
  ESP32 Smartwatch - UI + Sensor Test
  ===================================
  Hardware:
    ESP32 DevKit
    Main I2C bus  (SDA=21, SCL=22): SSD1306 128x64 OLED (0x3C), MPU6050 (0x68)
    MAX I2C bus   (SDA=25, SCL=26): MAX30102/MAX30105 (0x57)
      The MAX gets its own bus because many MAX30102 modules pull SDA/SCL
      up to 1.8V, which corrupts a bus shared with 3.3V devices.
      Set MAX_SEPARATE_BUS to 0 to put it back on the main bus.
    BTN_NEXT -> GPIO32 to GND   (internal pull-up, active LOW)
    BTN_PREV -> GPIO33 to GND   (internal pull-up, active LOW)

  Screens (slide animation + page indicator bar at the bottom):
    0: Clock   - big time, date, weather (Open-Meteo, no API key), steps
    1: Steps   - big steps/goal, animated loading bar, kcal + km
    2: Heart   - pounding heart while waiting/measuring, live pulse wave,
                 then the measured BPM with a heart beating in sync

  Buttons:
    NEXT short  -> next screen          PREV short -> previous screen
    NEXT long   -> context action:
                   Clock: refresh time + weather
                   Steps: reset step count
                   Heart: restart measurement
    PREV long   -> jump to Clock screen
    Any press while the screen is off only wakes it.

  Libraries: Adafruit SSD1306, Adafruit GFX, MPU6050 (i2cdevlib / Electronic Cats),
             SparkFun MAX3010x. WiFi + HTTPClient come with the ESP32 core.
*/

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050.h>
#include "MAX30105.h"
#include "heartRate.h"

// Custom types used as function return values must be declared before the
// first function, because the Arduino IDE auto-inserts function prototypes there.
enum WxType { WX_CLEAR, WX_CLOUD, WX_FOG, WX_DRIZZLE, WX_RAIN, WX_SNOW, WX_STORM };
enum BtnEvent { BTN_NONE, BTN_SHORT, BTN_LONG };

// ======================= USER CONFIG =======================
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* TZ_INFO   = "IST-5:30";        // POSIX timezone string
const bool  USE_24H   = true;

// Set these to your location for weather
const float WEATHER_LAT = 0.0f;
const float WEATHER_LON = 0.0f;
const unsigned long WEATHER_REFRESH_MS = 15UL * 60UL * 1000UL;

const uint32_t STEP_GOAL     = 10000;
const float    KCAL_PER_STEP = 0.04f;   // rough average for ~70 kg adult
const float    STRIDE_M      = 0.75f;

// ======================= PINS / TIMING =======================
#define PIN_SDA   21
#define PIN_SCL   22
#define BTN_NEXT  32
#define BTN_PREV  33

// 100 kHz is what worked in sensor_test.ino. At 100 kHz a full OLED frame
// takes ~90 ms, so animations run at ~12 fps. Once the bus is stable
// (short wires, 4.7k pull-ups) you can try 400000 for ~30 fps.
#define I2C_CLOCK 100000UL

// Set to 0 to test without WiFi (helps check for power-related glitches)
#define ENABLE_WIFI 1

// MAX30102 on its own I2C bus (ESP32 second I2C controller)
#define MAX_SEPARATE_BUS 1
#define MAX_SDA 25
#define MAX_SCL 26
#if MAX_SEPARATE_BUS
  #define MAX_WIRE Wire1
#else
  #define MAX_WIRE Wire
#endif

#define SCREEN_W  128
#define SCREEN_H  64
#define NUM_SCREENS 3
#define INDICATOR_Y 58            // content must stay above this line

#define FRAME_MS           (I2C_CLOCK >= 400000UL ? 33 : 80)
#define TRANS_MS           260    // screen slide duration
#define LONG_PRESS_MS      800
#define DEBOUNCE_MS        25
#define SCREEN_TIMEOUT_MS  0      // 0 = never sleep (testing). e.g. 15000 for 15 s

// ======================= OBJECTS =======================
// Last two args: I2C clock during and after display transfers
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1, I2C_CLOCK, I2C_CLOCK);
MPU6050  mpu(0x68);
MAX30105 maxSensor;

bool oled_ok = false, mpu_ok = false, max_ok = false;
uint32_t busRecoveries = 0;

// ======================= BUTTONS =======================

struct Button {
  uint8_t pin;
  bool stable = HIGH, lastRead = HIGH, longFired = false;
  unsigned long lastChange = 0, pressStart = 0;
  Button(uint8_t p) : pin(p) {}
};

Button btnNext(BTN_NEXT);
Button btnPrev(BTN_PREV);

BtnEvent pollButton(Button& b) {
  bool r = digitalRead(b.pin);
  unsigned long now = millis();
  if (r != b.lastRead) { b.lastRead = r; b.lastChange = now; }

  if (now - b.lastChange < DEBOUNCE_MS || r == b.stable) {
    // No confirmed change: check for long press while held
    if (b.stable == LOW && !b.longFired && now - b.pressStart >= LONG_PRESS_MS) {
      b.longFired = true;
      return BTN_LONG;
    }
    return BTN_NONE;
  }

  b.stable = r;
  if (r == LOW) {                 // pressed
    b.pressStart = now;
    b.longFired = false;
    return BTN_NONE;
  }
  return b.longFired ? BTN_NONE : BTN_SHORT;   // released
}

// ======================= STEP COUNTER =======================
uint32_t stepCount = 0;

const float LPF_ALPHA  = 0.2f;
const int   WINDOW     = 50;       // 1 s at 50 Hz
const float MIN_P2P_G  = 0.15f;
const unsigned long MIN_STEP_MS = 250, MAX_STEP_MS = 2000;
const int   CONFIRM_STEPS = 4;

float filtMag = 1.0f, prevFilt = 1.0f;
float winMin = 10.0f, winMax = -10.0f, threshold = 1.0f, sensitivity = 0.0f;
int   winCount = 0, pendingSteps = 0;
unsigned long lastStepTime = 0;

void updateStepCounter(float ax, float ay, float az) {
  float mag = sqrt(ax * ax + ay * ay + az * az);
  filtMag += LPF_ALPHA * (mag - filtMag);

  if (filtMag < winMin) winMin = filtMag;
  if (filtMag > winMax) winMax = filtMag;
  if (++winCount >= WINDOW) {
    threshold   = (winMax + winMin) / 2.0f;
    sensitivity = winMax - winMin;
    winMin = 10.0f; winMax = -10.0f; winCount = 0;
  }

  if (prevFilt > threshold && filtMag <= threshold && sensitivity > MIN_P2P_G) {
    unsigned long now = millis();
    unsigned long dt = now - lastStepTime;
    if (dt >= MIN_STEP_MS) {
      if (dt <= MAX_STEP_MS) {
        if (pendingSteps >= CONFIRM_STEPS) stepCount++;
        else if (++pendingSteps == CONFIRM_STEPS) stepCount += CONFIRM_STEPS;
      } else {
        pendingSteps = 1;
      }
      lastStepTime = now;
    }
  }
  prevFilt = filtMag;
}

void sampleAccel() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  // Default range +-2g = 16384 LSB/g
  updateStepCounter(ax / 16384.0f, ay / 16384.0f, az / 16384.0f);
}

// ======================= HEART RATE =======================
enum HrState { HR_OFF, HR_WAIT, HR_MEASURING, HR_RESULT };
HrState hrState = HR_OFF;

const long FINGER_THRESHOLD   = 50000;
const unsigned long MEASURE_MIN_MS = 7000;
const int  MEASURE_MIN_BEATS  = 6;
const byte RATE_SIZE          = 8;

byte  rates[RATE_SIZE];
byte  rateSpot = 0, rateCount = 0;
int   beatAvg = 0, lastBpm = 0, validBeats = 0;
bool  firstBeat = true;
unsigned long lastBeat = 0, measureStart = 0;

// Pulse waveform (1 px per sample, ~33 Hz)
#define WAVE_LEN 76
int16_t wave[WAVE_LEN];
int   waveIdx = 0, dsCount = 0;
float irDC = 0;
bool  dcInit = false;

// Heart animation
float realPulse = 0;                 // 1.0 on a detected beat, decays to 0
unsigned long synthStart = 0;        // phase origin for the fake heartbeat

void resetHeartBuffers() {
  rateSpot = rateCount = 0;
  beatAvg = validBeats = 0;
  firstBeat = true;
  realPulse = 0;
  memset(wave, 0, sizeof(wave));
  dcInit = false;
}

void startMeasurement() {
  resetHeartBuffers();
  measureStart = millis();
  lastBeat = millis();
  hrState = HR_MEASURING;
}

void addBeat(int bpm) {
  // Reject outliers once we have a baseline
  if (rateCount >= 3 && abs(bpm - beatAvg) > beatAvg / 4) return;

  rates[rateSpot++] = (byte)bpm;
  rateSpot %= RATE_SIZE;
  if (rateCount < RATE_SIZE) rateCount++;

  int sum = 0;
  for (byte i = 0; i < rateCount; i++) sum += rates[i];
  beatAvg = sum / rateCount;
  validBeats++;
  realPulse = 1.0f;
}

void processIR(long ir) {
  // --- waveform: remove DC, invert (PPG dips on each beat), downsample ---
  if (!dcInit) { irDC = ir; dcInit = true; }
  irDC += (ir - irDC) * 0.02f;
  if (++dsCount >= 3) {
    dsCount = 0;
    long v = -(long)(ir - irDC);
    wave[waveIdx] = (int16_t)constrain(v, -32000L, 32000L);
    waveIdx = (waveIdx + 1) % WAVE_LEN;
  }

  bool finger = ir > FINGER_THRESHOLD;

  switch (hrState) {
    case HR_WAIT:
      if (finger) startMeasurement();
      break;

    case HR_MEASURING:
    case HR_RESULT:
      if (!finger) {
        if (hrState == HR_RESULT && beatAvg > 0) lastBpm = beatAvg;
        hrState = HR_WAIT;
        synthStart = millis();
        return;
      }
      if (checkForBeat(ir)) {
        unsigned long now = millis();
        long delta = now - lastBeat;
        lastBeat = now;
        if (firstBeat) { firstBeat = false; break; }   // no valid interval yet
        int bpm = 60000 / delta;
        if (bpm >= 40 && bpm <= 200) addBeat(bpm);
      }
      if (hrState == HR_MEASURING &&
          millis() - measureStart >= MEASURE_MIN_MS &&
          validBeats >= MEASURE_MIN_BEATS) {
        hrState = HR_RESULT;
        lastBpm = beatAvg;
      }
      break;

    default:
      break;
  }
}

void updateHeartSensor() {
  if (hrState == HR_OFF || !max_ok) return;
  maxSensor.check();                    // non-blocking FIFO read
  while (maxSensor.available()) {
    processIR(maxSensor.getFIFOIR());
    maxSensor.nextSample();
  }
}

float measureProgress() {
  float tp = (millis() - measureStart) / (float)MEASURE_MIN_MS;
  float bp = validBeats / (float)MEASURE_MIN_BEATS;
  return constrain(min(tp, bp), 0.0f, 1.0f);
}

// Heart pulse amount (0..1) used to scale the heart icon
float heartPulseAmount() {
  bool useReal = (hrState == HR_RESULT) || (hrState == HR_MEASURING && validBeats > 0);
  if (useReal) return realPulse;

  // Synthetic "lub-dub" at ~70 bpm while waiting / before first beat
  unsigned long phase = (millis() - synthStart) % 857;
  if (phase < 120) return 1.0f - phase / 120.0f;
  if (phase >= 180 && phase < 280) return 0.6f * (1.0f - (phase - 180) / 100.0f);
  return 0.0f;
}

// ======================= WEATHER / NETWORK =======================
struct Weather {
  bool  valid = false;
  float tempC = 0;
  int   code  = 0;
} weather;

enum NetState { NET_CONNECTING, NET_READY, NET_OFFLINE };
NetState netState = NET_OFFLINE;
unsigned long wifiStart = 0, lastWeatherAttempt = 0, lastWifiRetry = 0;
bool weatherForce = false;


WxType wxType(int c) {
  if (c <= 1)  return WX_CLEAR;
  if (c <= 3)  return WX_CLOUD;
  if (c <= 48) return WX_FOG;
  if (c <= 57) return WX_DRIZZLE;
  if (c <= 67 || (c >= 80 && c <= 82)) return WX_RAIN;
  if (c <= 77 || c == 85 || c == 86)   return WX_SNOW;
  return WX_STORM;
}

const char* wxText(int c) {
  switch (wxType(c)) {
    case WX_CLEAR:   return "Clear";
    case WX_CLOUD:   return "Cloudy";
    case WX_FOG:     return "Fog";
    case WX_DRIZZLE: return "Drizzle";
    case WX_RAIN:    return "Rain";
    case WX_SNOW:    return "Snow";
    default:         return "Storm";
  }
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // smaller current spikes on the 3.3V rail
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiStart = millis();
  netState = NET_CONNECTING;
}

bool fetchWeather() {
  char url[180];
  snprintf(url, sizeof(url),
    "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
    "&current=temperature_2m,weather_code", WEATHER_LAT, WEATHER_LON);

  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(4000);
  if (!http.begin(url)) return false;

  int code = http.GET();
  if (code != 200) { http.end(); return false; }
  String body = http.getString();
  http.end();

  // Minimal parsing, no JSON library needed
  int c = body.indexOf("\"current\":{");
  if (c < 0) return false;
  int t = body.indexOf("\"temperature_2m\":", c);
  int w = body.indexOf("\"weather_code\":", c);
  if (t < 0 || w < 0) return false;

  weather.tempC = body.substring(t + 17).toFloat();
  weather.code  = body.substring(w + 15).toInt();
  weather.valid = true;
  Serial.printf("Weather: %.1f C, code %d\n", weather.tempC, weather.code);
  return true;
}

void updateNetwork(bool busy) {
  unsigned long now = millis();

  switch (netState) {
    case NET_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi OK: %s\n", WiFi.localIP().toString().c_str());
        configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
        netState = NET_READY;
        weatherForce = true;
      } else if (now - wifiStart > 20000) {
        Serial.println("WiFi failed, retrying later");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        netState = NET_OFFLINE;
        lastWifiRetry = now;
      }
      break;

    case NET_READY:
      if (WiFi.status() != WL_CONNECTED) {
        netState = NET_CONNECTING;       // ESP32 auto-reconnects
        wifiStart = now;
        break;
      }
      if (busy) break;                   // don't block during animations
      if (weatherForce ||
          (!weather.valid && now - lastWeatherAttempt > 30000) ||
          (now - lastWeatherAttempt > WEATHER_REFRESH_MS)) {
        weatherForce = false;
        lastWeatherAttempt = now;
        fetchWeather();
      }
      break;

    case NET_OFFLINE:
#if ENABLE_WIFI
      if (now - lastWifiRetry > 5UL * 60UL * 1000UL) startWiFi();
#endif
      break;
  }
}

bool getTimeNow(struct tm& t) {
  time_t now = time(nullptr);
  localtime_r(&now, &t);
  return now > 1700000000;
}

// ======================= UI STATE =======================
int  currentScreen = 0, prevScreen = 0, transDir = 1;
bool transitioning = false;
unsigned long transStart = 0;

float indicatorX = -1;
float shownProgress = 0;             // animated step bar
float frameDt = 0.033f;              // seconds since last frame

bool screenOn = true;
unsigned long lastActivity = 0;

float easeOutCubic(float t) { t = 1.0f - t; return 1.0f - t * t * t; }

// Fraction to move toward a target this frame, independent of frame rate
float approach(float ratePerSec) { return 1.0f - exp(-ratePerSec * frameDt); }

void onScreenEnter(int s) {
  if (s == 1) shownProgress = 0;     // replay the loading animation
  if (s == 2 && max_ok) {
    maxSensor.wakeUp();
    maxSensor.clearFIFO();
    resetHeartBuffers();
    synthStart = millis();
    hrState = HR_WAIT;
  }
}

void onScreenExit(int s) {
  if (s == 2 && max_ok) {
    if (hrState == HR_RESULT && beatAvg > 0) lastBpm = beatAvg;
    hrState = HR_OFF;
    maxSensor.shutDown();            // LEDs off, saves power
  }
}

void goToScreen(int target, int dir) {
  if (target == currentScreen) return;
  onScreenExit(currentScreen);
  prevScreen = currentScreen;
  currentScreen = target;
  onScreenEnter(currentScreen);
  transDir = dir;
  transStart = millis();
  transitioning = true;
}

// ======================= DRAW HELPERS =======================
void printRight(int rightX, int y, const char* s) {
  display.setCursor(rightX - (int)strlen(s) * 6, y);
  display.print(s);
}

void printCentered(int cx, int y, const char* s, int size = 1) {
  display.setTextSize(size);
  display.setCursor(cx - (int)strlen(s) * 6 * size / 2, y);
  display.print(s);
}

void drawCloud(int x, int y) {
  display.fillCircle(x + 3, y + 4, 2, SSD1306_WHITE);
  display.fillCircle(x + 6, y + 3, 3, SSD1306_WHITE);
  display.fillRect(x + 1, y + 4, 8, 3, SSD1306_WHITE);
}

void drawWeatherIcon(int x, int y, int code) {   // ~10x9 px
  switch (wxType(code)) {
    case WX_CLEAR:
      display.fillCircle(x + 4, y + 4, 2, SSD1306_WHITE);
      display.drawPixel(x + 4, y,     SSD1306_WHITE);
      display.drawPixel(x + 4, y + 8, SSD1306_WHITE);
      display.drawPixel(x,     y + 4, SSD1306_WHITE);
      display.drawPixel(x + 8, y + 4, SSD1306_WHITE);
      display.drawPixel(x + 1, y + 1, SSD1306_WHITE);
      display.drawPixel(x + 7, y + 1, SSD1306_WHITE);
      display.drawPixel(x + 1, y + 7, SSD1306_WHITE);
      display.drawPixel(x + 7, y + 7, SSD1306_WHITE);
      break;
    case WX_CLOUD:
      drawCloud(x, y + 1);
      break;
    case WX_FOG:
      display.drawFastHLine(x, y + 2, 9, SSD1306_WHITE);
      display.drawFastHLine(x + 1, y + 4, 8, SSD1306_WHITE);
      display.drawFastHLine(x, y + 6, 9, SSD1306_WHITE);
      break;
    case WX_DRIZZLE:
    case WX_RAIN:
      drawCloud(x, y);
      display.drawPixel(x + 2, y + 8, SSD1306_WHITE);
      display.drawPixel(x + 5, y + 8, SSD1306_WHITE);
      display.drawPixel(x + 8, y + 8, SSD1306_WHITE);
      break;
    case WX_SNOW:
      drawCloud(x, y);
      display.drawPixel(x + 3, y + 8, SSD1306_WHITE);
      display.drawPixel(x + 7, y + 8, SSD1306_WHITE);
      break;
    default:  // storm
      drawCloud(x, y);
      display.drawLine(x + 5, y + 6, x + 4, y + 8, SSD1306_BLACK);
      display.drawLine(x + 5, y + 7, x + 4, y + 9, SSD1306_WHITE);
      break;
  }
}

void drawHeart(int cx, int cy, int r) {
  display.fillCircle(cx - r, cy, r, SSD1306_WHITE);
  display.fillCircle(cx + r, cy, r, SSD1306_WHITE);
  display.fillTriangle(cx - 2 * r, cy, cx + 2 * r, cy,
                       cx, cy + 2 * r + r / 2, SSD1306_WHITE);
}

void drawAnimatedHeart(int cx, int cy) {
  float p = heartPulseAmount();
  int r = 6 + (int)round(2.0f * p);
  drawHeart(cx, cy - r / 2, r);

  // Expanding ripple right after a beat
  if (p > 0.15f && p < 0.85f) {
    int rr = 14 + (int)((1.0f - p) * 8);
    display.drawCircle(cx, cy + 2, rr, SSD1306_WHITE);
  }
}

void drawWave(int x, int y, int w, int h) {
  int16_t mn = 32767, mx = -32768;
  for (int i = 0; i < WAVE_LEN; i++) {
    if (wave[i] < mn) mn = wave[i];
    if (wave[i] > mx) mx = wave[i];
  }
  long range = (long)mx - mn;
  if (range < 20) range = 20;

  int n = min(w, WAVE_LEN);
  int px = 0, py = 0;
  for (int i = 0; i < n; i++) {
    int idx = (waveIdx + WAVE_LEN - n + i) % WAVE_LEN;   // oldest -> newest
    int yy = y + h - 1 - (int)(((long)wave[idx] - mn) * (h - 1) / range);
    int xx = x + i;
    if (i > 0) display.drawLine(px, py, xx, yy, SSD1306_WHITE);
    px = xx; py = yy;
  }
}

// ======================= SCREENS =======================
void drawClockScreen(int ox) {
  struct tm t;
  bool ok = getTimeNow(t);
  char buf[24];

  // --- top row: date + weather ---
  display.setTextSize(1);
  display.setCursor(ox, 0);
  if (ok) {
    strftime(buf, sizeof(buf), "%a %d %b", &t);
    display.print(buf);
  } else {
    display.print(netState == NET_OFFLINE ? "No WiFi" : "Syncing...");
  }

  if (weather.valid) {
    snprintf(buf, sizeof(buf), "%d", (int)round(weather.tempC));
    int nw = strlen(buf) * 6;
    int x = ox + SCREEN_W - (nw + 4 + 6);             // number + degree + "C"
    display.setCursor(x, 0);
    display.print(buf);
    display.drawCircle(x + nw + 1, 1, 1, SSD1306_WHITE);  // degree sign
    display.setCursor(x + nw + 4, 0);
    display.print("C");
    drawWeatherIcon(x - 12, 0, weather.code);
  } else {
    printRight(ox + SCREEN_W, 0, "--");
  }
  display.drawFastHLine(ox, 10, SCREEN_W, SSD1306_WHITE);

  // --- big time with blinking colon ---
  display.setTextSize(3);
  display.setCursor(ox + 16, 16);
  if (ok) {
    int hr = t.tm_hour;
    if (!USE_24H) { hr %= 12; if (hr == 0) hr = 12; }
    snprintf(buf, sizeof(buf), "%02d%c%02d", hr, (t.tm_sec % 2 == 0) ? ':' : ' ', t.tm_min);
  } else {
    strcpy(buf, "--:--");
  }
  display.print(buf);

  display.setTextSize(1);
  if (ok) {
    snprintf(buf, sizeof(buf), "%02d", t.tm_sec);
    display.setCursor(ox + 110, 30);
    display.print(buf);
    if (!USE_24H) {
      display.setCursor(ox + 110, 18);
      display.print(t.tm_hour < 12 ? "AM" : "PM");
    }
  }

  // --- bottom row: steps + weather text ---
  display.setCursor(ox, 46);
  display.printf("%lu steps", (unsigned long)stepCount);
  if (weather.valid) printRight(ox + SCREEN_W, 46, wxText(weather.code));
}

void drawStepsScreen(int ox) {
  char buf[24];

  display.setTextSize(1);
  display.setCursor(ox, 0);
  display.print("STEPS");
  snprintf(buf, sizeof(buf), "%.2f km", stepCount * STRIDE_M / 1000.0f);
  printRight(ox + SCREEN_W, 0, buf);
  display.drawFastHLine(ox, 10, SCREEN_W, SSD1306_WHITE);

  // --- big steps + small /goal, centered together ---
  char sb[12], gb[12];
  snprintf(sb, sizeof(sb), "%lu", (unsigned long)stepCount);
  snprintf(gb, sizeof(gb), "/%lu", (unsigned long)STEP_GOAL);
  int w = strlen(sb) * 18 + strlen(gb) * 6;
  int x = ox + max(0, (SCREEN_W - w) / 2);
  display.setTextSize(3);
  display.setCursor(x, 14);
  display.print(sb);
  display.setTextSize(1);
  display.setCursor(x + strlen(sb) * 18, 28);
  display.print(gb);

  // --- loading bar ---
  int bx = ox + 2, by = 41, bw = SCREEN_W - 4, bh = 7;
  display.drawRoundRect(bx, by, bw, bh, 3, SSD1306_WHITE);
  int innerW = bw - 4;
  int fw = (int)(innerW * shownProgress);
  if (fw > 0) display.fillRect(bx + 2, by + 2, fw, bh - 4, SSD1306_WHITE);

  // shimmer band sweeping through the filled part
  if (fw > 8 && shownProgress < 0.999f) {
    int sx = (int)((millis() / 12) % (fw + 12)) - 6;
    for (int k = 0; k < 3; k++) {
      int px = sx + k;
      if (px >= 0 && px < fw)
        display.drawFastVLine(bx + 2 + px, by + 2, bh - 4, SSD1306_BLACK);
    }
  }

  // --- bottom row: kcal + percent ---
  display.setCursor(ox, 50);
  display.printf("%.1f kcal", stepCount * KCAL_PER_STEP);

  bool goalHit = stepCount >= STEP_GOAL;
  if (goalHit) {
    if ((millis() / 400) % 2) printRight(ox + SCREEN_W, 50, "GOAL!");
  } else {
    snprintf(buf, sizeof(buf), "%d%%", (int)(100.0f * stepCount / STEP_GOAL));
    printRight(ox + SCREEN_W, 50, buf);
  }
}

void drawHeartScreen(int ox) {
  const int hx = ox + 22, hy = 22;     // heart centre (left column)
  const int rx = ox + 50;              // right column start
  char buf[24];

  if (!max_ok) {
    printCentered(ox + SCREEN_W / 2, 20, "HR sensor");
    printCentered(ox + SCREEN_W / 2, 32, "not found");
    return;
  }

  drawAnimatedHeart(hx, hy);
  display.setTextSize(1);

  switch (hrState) {
    case HR_WAIT:
      display.setCursor(rx, 6);  display.print("Place your");
      display.setCursor(rx, 16); display.print("finger on");
      display.setCursor(rx, 26); display.print("the sensor");
      if (lastBpm > 0) {
        snprintf(buf, sizeof(buf), "Last %d bpm", lastBpm);
        display.setCursor(rx, 44);
        display.print(buf);
      }
      printCentered(hx, 48, "--");
      break;

    case HR_MEASURING: {
      int dots = (millis() / 350) % 4;
      display.setCursor(rx, 0);
      display.print("Measuring");
      for (int i = 0; i < dots; i++) display.print('.');

      drawWave(rx, 12, SCREEN_W - 52, 26);

      int pw = SCREEN_W - 52;
      display.drawRect(rx, 42, pw, 5, SSD1306_WHITE);
      display.fillRect(rx + 1, 43, (int)((pw - 2) * measureProgress()), 3, SSD1306_WHITE);

      display.setCursor(rx, 50);
      display.print("Hold still");
      printCentered(hx, 48, "...");
      break;
    }

    case HR_RESULT: {
      snprintf(buf, sizeof(buf), "%d", beatAvg);
      display.setTextSize(3);
      display.setCursor(rx, 4);
      display.print(buf);
      display.setTextSize(1);
      display.setCursor(rx + strlen(buf) * 18 + 3, 18);
      display.print("BPM");

      drawWave(rx, 32, SCREEN_W - 52, 22);

      const char* zone = beatAvg < 60 ? "Low" : beatAvg < 100 ? "Rest"
                       : beatAvg < 140 ? "Active" : "High";
      printCentered(hx, 48, zone);
      break;
    }

    default:
      break;
  }
}

void drawScreen(int s, int ox) {
  switch (s) {
    case 0: drawClockScreen(ox); break;
    case 1: drawStepsScreen(ox); break;
    case 2: drawHeartScreen(ox); break;
  }
}

// Bottom page indicator: dotted track + sliding pill
void drawIndicator() {
  display.fillRect(0, INDICATOR_Y, SCREEN_W, SCREEN_H - INDICATOR_Y, SSD1306_BLACK);

  const int segW = 16, gap = 4;
  int total = NUM_SCREENS * segW + (NUM_SCREENS - 1) * gap;
  int x0 = (SCREEN_W - total) / 2;

  for (int x = x0; x < x0 + total; x += 2)
    display.drawPixel(x, 61, SSD1306_WHITE);

  float target = x0 + currentScreen * (segW + gap);
  if (indicatorX < 0) indicatorX = target;
  indicatorX += (target - indicatorX) * approach(12.0f);
  if (fabs(target - indicatorX) < 0.5f) indicatorX = target;

  display.fillRoundRect((int)round(indicatorX), 59, segW, 5, 2, SSD1306_WHITE);
}

// ======================= RENDER / ANIMATION =======================
void updateAnimations(float dt) {
  float target = min(1.0f, stepCount / (float)STEP_GOAL);
  shownProgress += (target - shownProgress) * approach(2.5f);
  if (fabs(target - shownProgress) < 0.001f) shownProgress = target;

  realPulse = max(0.0f, realPulse - dt / 0.18f);   // ~180 ms decay
}

void renderFrame() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  if (transitioning) {
    float t = (millis() - transStart) / (float)TRANS_MS;
    if (t >= 1.0f) {
      transitioning = false;
      drawScreen(currentScreen, 0);
    } else {
      int off = (int)(easeOutCubic(t) * SCREEN_W);
      drawScreen(prevScreen, -transDir * off);
      drawScreen(currentScreen, transDir * (SCREEN_W - off));
    }
  } else {
    drawScreen(currentScreen, 0);
  }

  drawIndicator();
  display.display();
}

// ======================= SLEEP / INPUT =======================
void sleepScreen() {
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  screenOn = false;
  onScreenExit(currentScreen);
}

void wakeScreen() {
  display.ssd1306_command(SSD1306_DISPLAYON);
  screenOn = true;
  lastActivity = millis();
  onScreenEnter(currentScreen);
}

void handleInput() {
  BtnEvent n = pollButton(btnNext);
  BtnEvent p = pollButton(btnPrev);
  if (n == BTN_NONE && p == BTN_NONE) return;

  if (!screenOn) { wakeScreen(); return; }     // first press only wakes
  lastActivity = millis();

  if (n == BTN_SHORT) goToScreen((currentScreen + 1) % NUM_SCREENS, +1);
  if (p == BTN_SHORT) goToScreen((currentScreen + NUM_SCREENS - 1) % NUM_SCREENS, -1);
  if (p == BTN_LONG)  goToScreen(0, -1);

  if (n == BTN_LONG) {
    switch (currentScreen) {
      case 0:
        if (netState == NET_READY) {
          configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
          weatherForce = true;
        } else {
          startWiFi();
        }
        break;
      case 1:
        stepCount = 0;
        shownProgress = 0;
        break;
      case 2:
        if (max_ok) { resetHeartBuffers(); synthStart = millis(); hrState = HR_WAIT; }
        break;
    }
  }
}

// ======================= BOOT / SENSOR TEST =======================
bool i2cPresentOn(TwoWire& bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

bool i2cPresent(uint8_t addr) {
  return i2cPresentOn(Wire, addr);
}

void scanBus(TwoWire& bus, const char* name) {
  Serial.printf("I2C scan (%s):\n", name);
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresentOn(bus, a)) {
      Serial.printf("  found 0x%02X\n", a);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found - check wiring / power");
}

void scanI2C() {
  scanBus(Wire, "main 21/22");
#if MAX_SEPARATE_BUS
  scanBus(Wire1, "MAX 25/26");
#endif
}

// Frees a bus where a device is holding SDA low, then restarts Wire
void recoverI2C() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SCL, HIGH);
  for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  }
  // STOP condition
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(10);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(10);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK);
}

// Called every 2 s: if the OLED stops answering, recover the bus
void checkBusHealth() {
  if (!oled_ok || i2cPresent(0x3C)) return;

  Serial.println("I2C: OLED not responding, recovering bus...");
  recoverI2C();
  if (!i2cPresent(0x3C)) {
    Serial.println("I2C: recovery failed");
    return;
  }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false);
  display.setTextWrap(false);
  if (!screenOn) display.ssd1306_command(SSD1306_DISPLAYOFF);
  if (mpu_ok) { mpu.initialize(); mpu.setSleepEnabled(false); }
  busRecoveries++;
  Serial.println("I2C: recovered");
}

void runSensorTest() {
  scanI2C();

  // ---- OLED ---- (periphBegin=false: keep our Wire pins/clock)
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
    oled_ok = true;
    display.setTextWrap(false);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    printCentered(SCREEN_W / 2, 16, "ESP WATCH", 2);
    printCentered(SCREEN_W / 2, 40, "starting...");
    display.display();
  }
  Serial.printf("OLED: %s\n", oled_ok ? "PASS" : "FAIL");
  delay(1000);

  // ---- MPU6050 (validated with real data, works with clones) ----
  bool mpuSeen = i2cPresent(0x68);
  Serial.printf("MPU6050 at 0x68: %s\n", mpuSeen ? "yes" : "NO");
  for (int attempt = 0; attempt < 3 && !mpu_ok; attempt++) {
    mpu.initialize();
    mpu.setSleepEnabled(false);
    delay(100);
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    Serial.printf("  MPU try %d: ax=%d ay=%d az=%d\n", attempt + 1, ax, ay, az);
    mpu_ok = (abs(ax) > 50 || abs(ay) > 50 || abs(az) > 50);
    if (!mpu_ok) delay(200);
  }
  Serial.printf("MPU6050: %s\n", mpu_ok ? "PASS" : "FAIL");
  delay(500);

  // ---- MAX30102 ---- (same speed as the rest of the bus)
  delay(500);   // from sensor_test.ino: prevents bus lock
  Serial.printf("MAX3010x at 0x57: %s\n", i2cPresentOn(MAX_WIRE, 0x57) ? "yes" : "NO");
  if (maxSensor.begin(MAX_WIRE, I2C_CLOCK)) {
    // brightness, avg, ledMode(2=red+IR), rate, pulseWidth, adcRange -> ~100 Hz
    maxSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    delay(200);
    max_ok = true;
  } else {
    // begin() failed on the part-ID check; still try to switch the LEDs off
    maxSensor.shutDown();
  }
  if (max_ok) maxSensor.shutDown();    // only on while the heart screen is open
  MAX_WIRE.setClock(I2C_CLOCK);        // make sure nothing changed the bus speed
  Serial.printf("MAX3010x: %s\n", max_ok ? "PASS" : "FAIL");

  // ---- Results screen ----
  if (oled_ok) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Sensor check");
    display.drawFastHLine(0, 10, SCREEN_W, SSD1306_WHITE);
    display.setCursor(0, 16);
    display.printf("OLED     %s", oled_ok ? "PASS" : "FAIL");
    display.setCursor(0, 28);
    display.printf("MPU6050  %s", mpu_ok ? "PASS" : "FAIL");
    display.setCursor(0, 40);
    display.printf("MAX3010x %s", max_ok ? "PASS" : "FAIL");
    display.display();
    delay(2000);
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_PREV, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_CLOCK);
#if MAX_SEPARATE_BUS
  Wire1.begin(MAX_SDA, MAX_SCL);
  Wire1.setClock(I2C_CLOCK);
#endif
  delay(200);

  Serial.println("\n===== ESP WATCH UI TEST =====");
  runSensorTest();

#if ENABLE_WIFI
  startWiFi();
#endif
  onScreenEnter(currentScreen);
  lastActivity = millis();
}

void loop() {
  unsigned long now = millis();

  // Sensors
  static unsigned long lastAccel = 0;
  if (mpu_ok && now - lastAccel >= 20) {        // 50 Hz
    lastAccel = now;
    sampleAccel();
  }
  updateHeartSensor();

  // Input + network
  handleInput();
  updateNetwork(transitioning);

  // Keep screen awake while a heart measurement is running
  if (hrState == HR_MEASURING || hrState == HR_RESULT) lastActivity = now;
  if (SCREEN_TIMEOUT_MS > 0 && screenOn && now - lastActivity > SCREEN_TIMEOUT_MS)
    sleepScreen();

  static unsigned long lastBusCheck = 0;
  if (now - lastBusCheck >= 2000) {
    lastBusCheck = now;
    checkBusHealth();
  }

  // Rendering
  static unsigned long lastFrame = 0;
  if (screenOn && oled_ok && now - lastFrame >= FRAME_MS) {
    float dt = (now - lastFrame) / 1000.0f;
    if (dt > 0.2f) dt = 0.2f;
    lastFrame = now;
    frameDt = dt;
    updateAnimations(dt);
    renderFrame();
    updateHeartSensor();                        // drain FIFO after the I2C burst
  }

  // Midnight step reset
  static unsigned long lastDayCheck = 0;
  static int lastDay = -1;
  if (now - lastDayCheck >= 1000) {
    lastDayCheck = now;
    struct tm t;
    if (getTimeNow(t)) {
      if (lastDay != -1 && t.tm_mday != lastDay) stepCount = 0;
      lastDay = t.tm_mday;
    }
  }

  // Serial debug, 1 Hz
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 1000) {
    lastDebug = now;
    Serial.printf("screen=%d steps=%lu hrState=%d bpm=%d net=%d mag=%.2fg busFix=%lu\n",
                  currentScreen, (unsigned long)stepCount, hrState, beatAvg,
                  netState, filtMag, (unsigned long)busRecoveries);
  }
}
