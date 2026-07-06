// =============================================================
//  CAD Glove — Mouse mode + Clutch (button remap, no gestures)
//
//  Normal mode (clutch OFF): tilt-to-cursor, D0/D3 are
//  left-click / right-click, D1 is tare, D6 is Enter/Exit.
//
//  Clutch mode (tap D7 to toggle ON/OFF): cursor freezes, and:
//    Tap D0           -> Undo (Ctrl+Z)
//    Tap D3           -> Redo (Ctrl+Y)
//    TILT left/right  -> continuous zoom in/out
//    TILT up/down     -> Zoom to fit ('F') / View normal ('N') — one-shot,
//                         sustained-hold trigger, re-arms on return to neutral
//
//  !! TARE (D1) IS CURRENTLY DISABLED IN FIRMWARE !!
//  Re-enable once confirmed working: find "TARE TEMPORARILY DISABLED"
//  in loop() and delete those two lines.
//
//  D6 (Enter/Exit) has a startup grace period (ENTEREXITGRACE_MS) to
//  prevent the UART TX boot flap from registering as a phantom press.
//
//  Zoom safely reuses tilt because cursor tracking is fully off
//  while clutched — the two can never read the sensor at the same
//  time, so there's no cross-talk by construction, not by tuning.
//
//  Requires: ESP32-BLE-Combo library in lib/ (manual install,
//  see chat — avoids git/registry issues).
//
//  ---- CHANGES IN THIS VERSION (search for "FIX:") ----
//  1. FIX(hysteresis): up/down triggers now re-arm at a level BELOW the
//     fire threshold (clutchRollRearmDeg, default = half the threshold)
//     instead of borrowing zoomTiltDeadzoneDeg (20°), which was ABOVE the
//     15° fire threshold and caused repeat-firing every ~150ms when the
//     hand was held steady in the 15–20° band.
//  2. FIX(blocking serial): config is now read into a buffer one byte at
//     a time, non-blocking. readStringUntil() could stall the whole loop
//     (cursor, debounce, everything) for up to 1s on a partial line.
//  3. FIX(zoom invert): added invertZoom, settable over Web Serial, since
//     scroll-to-zoom direction differs between CAD packages. No reflash
//     needed at the demo table.
//  4. FIX(baseline noise): clutch-entry (and tare) baselines are now an
//     average of several quick reads instead of one raw sample, so a
//     slightly-moving hand at the moment of the press doesn't bake noise
//     into the reference angles.
// =============================================================

#include <Arduino.h>
#include <Wire.h>
#include <BleCombo.h>
#include <ArduinoJson.h>
#include <math.h>

// ---------------- Pin assignments ----------------
const int PIN_LEFT_CLICK  = D0;
const int PIN_RIGHT_CLICK = D3;
const int PIN_TARE        = D1;
const int PIN_ENTER_EXIT  = D6;   // tap = Enter, hold 2s = Escape. NOTE: D6 is UART TX —
                                   // startup grace period prevents boot-flap phantom press.
const int PIN_CLUTCH      = D7;   // tap to toggle clutch mode

// ---------------- MPU6050 (index finger) ----------------
const uint8_t MPU_ADDR         = 0x68;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ---------------- Zoom (tilt while clutched — mutually exclusive with cursor, zero cross-talk) ----------------
// One scroll tick per interval while tilted past the deadzone — simple,
// predictable rate instead of continuous accumulation (which made small
// sensitivity numbers hard to reason about since hold-duration also mattered).
float zoomTiltDeadzoneDeg = 20.0f;
unsigned long zoomTickIntervalMs = 250;  // larger = slower zoom, tune this directly
int invertZoom = 1;                      // FIX: flip to -1 if zoom direction is backwards in your CAD app
unsigned long lastZoomTick = 0;
float clutchTiltBaseline = 0.0f;   // pitch baseline (left/right tilt -> continuous zoom)
float clutchRollBaseline = 0.0f;   // roll baseline (up/down tilt -> discrete triggers)

// ---------------- Up/down discrete triggers (sustained tilt, not a burst —
// same reliable pattern as the "palm up" idea, arm/disarm on return to neutral) ----
float clutchUpDownThreshold = 15.0f;  // degrees from clutch-entry baseline — FIRE level
float clutchRollRearmDeg    = 7.5f;   // FIX: RE-ARM level, must stay BELOW the fire level
                                      // (real hysteresis: fire high, re-arm low). The old code
                                      // re-armed at 20° — above the 15° fire level — so a hand
                                      // held steady at ~17° fired repeatedly.
unsigned long clutchUpDownHoldMs = 150;
int invertClutchRoll = 1;  // flip to -1 if up/down come out backwards
bool clutchRollArmed = true;
bool clutchRollHoldActive = false;
unsigned long clutchRollHoldStart = 0;
int clutchRollPendingDir = 0;

// ---------------- Enter/Exit button (D6) ----------------
// Tap = Enter, hold >= 2s = Escape.
// D6 is UART TX and flaps HIGH/LOW during the first ~1s of boot —
// ignore all D6 input until ENTEREXITGRACE_MS has elapsed to prevent
// a phantom press at startup.
const unsigned long ENTEREXITGRACE_MS = 1500;  // ignore D6 for this long after boot
const unsigned long ENTEREXIT_HOLD_MS = 2000;  // hold this long to fire Escape
bool enterExitGraceDone = false;
unsigned long enterExitPressStart = 0;
bool enterExitHoldFired = false;

// ---------------- Clutch toggle state ----------------
bool clutchActive = false;
unsigned long clutchPressStartTime = 0;
bool clutchEasterEggArmed = true;

// ---------------- Tuning ----------------
float sensitivity        = 0.2f;
int invertX = 1;
int invertY = -1;
float deadzoneDeg        = 3.0f;
float smoothingAlpha = 0.2f;
int maxCursorSpeed  = 20;
const unsigned long sampleMs   = 10;
const unsigned long debounceMs = 30;

float pitchOffset = 0.0f;
float rollOffset  = 0.0f;
float smoothedPitch = 0.0f;
float smoothedRoll  = 0.0f;
bool smoothingInitialized = false;
unsigned long lastSampleTime = 0;

float xRemainder = 0.0f;
float yRemainder = 0.0f;

// ---------------- Debounce helper ----------------
struct Button {
  int pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
};

Button leftBtn        = { PIN_LEFT_CLICK,  HIGH, HIGH, 0 };
Button rightBtn       = { PIN_RIGHT_CLICK, HIGH, HIGH, 0 };
Button tareBtn        = { PIN_TARE,        HIGH, HIGH, 0 };
Button enterExitBtn   = { PIN_ENTER_EXIT,  HIGH, HIGH, 0 };
Button clutchBtn      = { PIN_CLUTCH,      HIGH, HIGH, 0 };

bool leftPressed  = false;
bool rightPressed = false;

bool updateButtonPressEdge(Button &b, bool &heldOut) {
  bool reading = digitalRead(b.pin);
  bool pressEdge = false;
  if (reading != b.lastReading) b.lastChangeTime = millis();
  if ((millis() - b.lastChangeTime) > debounceMs && reading != b.stableState) {
    b.stableState = reading;
    if (b.stableState == LOW) pressEdge = true;
  }
  b.lastReading = reading;
  heldOut = (b.stableState == LOW);
  return pressEdge;
}

bool updateButtonReleaseEdge(Button &b, bool &heldOut) {
  bool reading = digitalRead(b.pin);
  bool releaseEdge = false;
  if (reading != b.lastReading) b.lastChangeTime = millis();
  if ((millis() - b.lastChangeTime) > debounceMs && reading != b.stableState) {
    bool wasPressed = (b.stableState == LOW);
    b.stableState = reading;
    if (wasPressed && b.stableState == HIGH) releaseEdge = true;
  }
  b.lastReading = reading;
  heldOut = (b.stableState == LOW);
  return releaseEdge;
}

// ---------------- MPU6050 raw read (with manual bus recovery) ----------------
void mpuWake();
int consecutiveI2CFailures = 0;
const int MAX_I2C_FAILURES_BEFORE_RESET = 5;

void resetI2CBus() {
  Wire.end();
  delay(10);
  Wire.begin();
  Wire.setClock(400000);
  mpuWake();
  consecutiveI2CFailures = 0;
}

int16_t readWord(uint8_t highReg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(highReg);
  uint8_t txStatus = Wire.endTransmission(false);
  uint8_t received = Wire.requestFrom((int)MPU_ADDR, 2, true);
  if (txStatus != 0 || received != 2) {
    consecutiveI2CFailures++;
    if (consecutiveI2CFailures >= MAX_I2C_FAILURES_BEFORE_RESET) resetI2CBus();
    return 0;
  }
  consecutiveI2CFailures = 0;
  return (Wire.read() << 8) | Wire.read();
}

void mpuWake() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
}

void getPitchRoll(float &pitch, float &roll, float &magnitude) {
  int16_t axRaw = readWord(REG_ACCEL_XOUT_H);
  int16_t ayRaw = readWord(REG_ACCEL_XOUT_H + 2);
  int16_t azRaw = readWord(REG_ACCEL_XOUT_H + 4);
  float ax = axRaw / 16384.0f;
  float ay = ayRaw / 16384.0f;
  float az = azRaw / 16384.0f;
  magnitude = sqrt(ax * ax + ay * ay + az * az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
  roll  = atan2(ay, az) * 180.0f / PI;
}

// FIX: averaged read for setting reference baselines. A single raw sample
// taken on a press edge bakes whatever wobble the hand had at that exact
// instant into the baseline; averaging a short burst smooths that out.
// 5 reads x 6ms ≈ 30ms — imperceptible on a button press.
void getPitchRollAveraged(float &pitch, float &roll, int samples = 5) {
  float pSum = 0, rSum = 0;
  int used = 0;
  for (int i = 0; i < samples; i++) {
    float p, r, mag;
    getPitchRoll(p, r, mag);
    if (fabs(mag - 1.0f) < 0.3f) {  // skip mid-shake samples entirely
      pSum += p;
      rSum += r;
      used++;
    }
    delay(6);
  }
  if (used > 0) {
    pitch = pSum / used;
    roll  = rSum / used;
  } else {
    // hand was moving too fast the whole time — fall back to one raw read
    float mag;
    getPitchRoll(pitch, roll, mag);
  }
}

// ---------------- Clutch keyboard actions — remappable macros ----------------
// Each macro is up to 3 keys pressed together. Defaults match the original
// behavior; all four can be changed live via Web Serial (see readSerialConfig).
struct Macro {
  uint8_t keys[3];
  int count;
};

Macro undoMacro       = { { KEY_LEFT_CTRL, 'z', 0 }, 2 };
Macro redoMacro       = { { KEY_LEFT_CTRL, 'y', 0 }, 2 };  // Windows Ctrl+Y — Mac is Cmd+Shift+Z
Macro zoomFitMacro    = { { 'f', 0, 0 }, 1 };
Macro viewNormalMacro = { { 'v', 'n', 0 }, 2 };

// Hold macros for left/right clutch buttons (OnShape shortcuts)
Macro sketchMacro     = { { KEY_LEFT_SHIFT, 's', 0 }, 2 };  // Shift+S = Sketch
Macro extrudeMacro    = { { KEY_LEFT_SHIFT, 'e', 0 }, 2 };  // Shift+E = Extrude

// Hold detection for left/right clutch buttons
const unsigned long CLUTCH_BTN_HOLD_MS = 600;  // hold this long to fire hold macro
unsigned long leftClutchPressStart  = 0;
unsigned long rightClutchPressStart = 0;
bool leftClutchHoldFired  = false;
bool rightClutchHoldFired = false;

void sendMacro(const char* label, Macro &m) {
  Serial.print("Clutch: ");
  Serial.println(label);
  for (int i = 0; i < m.count; i++) Keyboard.press(m.keys[i]);
  delay(20);
  Keyboard.releaseAll();
}

// ---------------- Easter egg: opens a URL via Win+R Run dialog ----------------
// Keyboard.print() types raw keycodes assuming US layout — on other layouts
// ':' and '/' land on different keys and come out garbled. Win+R (Run dialog)
// then Keyboard.print() has the same problem, BUT we can work around it by
// typing only the characters that ARE layout-safe (letters, digits) and
// sending the tricky ones (: / .) as explicit shifted/unshifted keycodes
// referenced by their USB HID usage ID, which are always the same key
// regardless of what character your OS prints from it.
//
// For Mac: Cmd+Space (Spotlight) still has the same layout problem.
// Simplest Mac fix is to set a custom Spotlight shortcut that accepts a
// plain URL — or just flip isMacDemoMachine and accept it may mangle on
// non-US layouts.
bool isMacDemoMachine = false;

// Type a single character in a layout-safe way.
// Letters and digits are always in the same physical position — safe to
// print directly. Punctuation used in URLs needs explicit keycode handling.
void typeChar(char c) {
  // Keyboard.print() handles all standard ASCII correctly including
  // punctuation — the original HID usage ID approach (0x33 etc.) was wrong
  // because Keyboard.press() expects ASCII, not raw HID codes.
  Keyboard.print(c);
  delay(8);
}

void typeString(const char* str) {
  for (int i = 0; str[i] != '\0'; i++) typeChar(str[i]);
}

void sendOpenURL(const char* url) {
  Serial.print("Easter egg: opening ");
  Serial.println(url);

  if (isMacDemoMachine) {
    // Mac: Cmd+Space opens Spotlight
    Keyboard.press(KEY_LEFT_GUI);
    Keyboard.press(' ');
    delay(150);
    Keyboard.releaseAll();
    delay(300);  // Spotlight takes a moment to appear
  } else {
    // Windows: Win+R opens Run dialog
    Keyboard.press(KEY_LEFT_GUI);
    Keyboard.press('r');
    delay(150);
    Keyboard.releaseAll();
    delay(600);  // Run dialog needs time to fully open before typing
  }

  typeString(url);
  delay(100);
  Keyboard.press(KEY_RETURN);
  delay(20);
  Keyboard.releaseAll();
}

// change this to whatever's funny
const char* EASTER_EGG_URL = "https://www.youtube.com/watch?v=dQw4w9WgXcQ";

// Turns a token like "ctrl", "shift", "enter", or a single character like
// "z" into the keycode Keyboard.press() expects.
uint8_t resolveKeyToken(String tok) {
  tok.toLowerCase();
  if (tok == "ctrl")   return KEY_LEFT_CTRL;
  if (tok == "shift")  return KEY_LEFT_SHIFT;
  if (tok == "alt")    return KEY_LEFT_ALT;
  if (tok == "gui" || tok == "cmd" || tok == "win") return KEY_LEFT_GUI;
  if (tok == "enter" || tok == "return") return KEY_RETURN;
  if (tok == "esc" || tok == "escape")   return KEY_ESC;
  if (tok == "tab")    return KEY_TAB;
  if (tok.length() >= 1) return (uint8_t)tok[0];
  return 0;
}

// ---------------- Web Serial config receiver ----------------
// FIX: now fully non-blocking. The old version used readStringUntil('\n'),
// which blocks up to Serial's timeout (default 1000ms!) whenever bytes
// arrive without a newline — a partial write or stray character from a
// serial monitor would freeze cursor tracking and button debounce for up
// to a full second. Now we drain whatever bytes are available each loop
// pass into a buffer and only parse once a complete line has arrived.
//
// Reads one JSON line at a time and updates any matching tuning variables.
// Only fields present in the JSON get touched — send just what changed.
// Example line the website sends: {"sensitivity":0.5,"deadzoneDeg":4}
const size_t SERIAL_BUF_SIZE = 1024;  // FIX: was 512; the site's full payload was 499 bytes — no margin
char serialBuf[SERIAL_BUF_SIZE];
size_t serialBufLen = 0;

void applyConfigLine(const char* line) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("Config JSON error: ");
    Serial.println(err.c_str());
    return;
  }

  if (doc["sensitivity"].is<float>())             sensitivity = doc["sensitivity"];
  if (doc["deadzoneDeg"].is<float>())             deadzoneDeg = doc["deadzoneDeg"];
  if (doc["smoothingAlpha"].is<float>())          smoothingAlpha = doc["smoothingAlpha"];
  if (doc["maxCursorSpeed"].is<int>())            maxCursorSpeed = doc["maxCursorSpeed"];
  if (doc["invertX"].is<int>())                   invertX = doc["invertX"];
  if (doc["invertY"].is<int>())                   invertY = doc["invertY"];
  if (doc["zoomTiltDeadzoneDeg"].is<float>())     zoomTiltDeadzoneDeg = doc["zoomTiltDeadzoneDeg"];
  if (doc["zoomTickIntervalMs"].is<unsigned long>()) zoomTickIntervalMs = doc["zoomTickIntervalMs"];
  if (doc["invertZoom"].is<int>())                invertZoom = doc["invertZoom"];          // FIX: new
  if (doc["clutchUpDownThreshold"].is<float>())   clutchUpDownThreshold = doc["clutchUpDownThreshold"];
  if (doc["clutchRollRearmDeg"].is<float>())      clutchRollRearmDeg = doc["clutchRollRearmDeg"];  // FIX: new
  if (doc["clutchUpDownHoldMs"].is<unsigned long>()) clutchUpDownHoldMs = doc["clutchUpDownHoldMs"];
  if (doc["invertClutchRoll"].is<int>())          invertClutchRoll = doc["invertClutchRoll"];

  // FIX: guard against a config that recreates the original hysteresis bug.
  // The re-arm level must sit below the fire level, or steady tilt repeats.
  if (clutchRollRearmDeg >= clutchUpDownThreshold) {
    clutchRollRearmDeg = clutchUpDownThreshold * 0.5f;
    Serial.println("Warning: clutchRollRearmDeg >= threshold, clamped to half");
  }

  // Macros — send an array of key names, e.g. {"macroUndo":["ctrl","z"]}
  const char* macroFields[] = { "macroUndo", "macroRedo", "macroSketch", "macroExtrude", "macroZoomFit", "macroViewNormal" };
  Macro* macroTargets[] = { &undoMacro, &redoMacro, &sketchMacro, &extrudeMacro, &zoomFitMacro, &viewNormalMacro };
  for (int m = 0; m < 4; m++) {
    if (doc[macroFields[m]].is<JsonArray>()) {
      JsonArray arr = doc[macroFields[m]].as<JsonArray>();
      int i = 0;
      for (JsonVariant v : arr) {
        if (i >= 3) break;
        macroTargets[m]->keys[i] = resolveKeyToken(v.as<String>());
        i++;
      }
      macroTargets[m]->count = i;
    }
  }

  Serial.println("Config updated");
}

void readSerialConfig() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (serialBufLen > 0) {
        serialBuf[serialBufLen] = '\0';
        applyConfigLine(serialBuf);
        serialBufLen = 0;
      }
    } else if (c != '\r') {
      if (serialBufLen < SERIAL_BUF_SIZE - 1) {
        serialBuf[serialBufLen++] = c;
      } else {
        serialBufLen = 0;  // line too long — junk it and resync on next newline
        Serial.println("Config line too long, discarded");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_LEFT_CLICK,  INPUT_PULLUP);
  pinMode(PIN_RIGHT_CLICK, INPUT_PULLUP);
  pinMode(PIN_TARE,        INPUT_PULLUP);
  pinMode(PIN_ENTER_EXIT,  INPUT_PULLUP);
  pinMode(PIN_CLUTCH,      INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);
  mpuWake();

  delay(50);
  getPitchRollAveraged(pitchOffset, rollOffset);  // FIX: averaged boot baseline

  Keyboard.begin();
  Mouse.begin();
}

void loop() {
  unsigned long now = millis();
  readSerialConfig();




  bool leftHeld, rightHeld, tareHeld, clutchHeldRaw, enterExitHeld;
  bool leftPress  = updateButtonPressEdge(leftBtn, leftHeld);
  bool rightPress = updateButtonPressEdge(rightBtn, rightHeld);
  bool tareReleaseEdge = updateButtonReleaseEdge(tareBtn, tareHeld);

  // TARE TEMPORARILY DISABLED — solder joint failed, pin may float/read
  // erratically. Force both to a safe "never triggered" state regardless
  // of what the disconnected pin actually reads. Remove these two lines
  // once D1 is confirmed working.
  tareReleaseEdge = false;
  tareHeld = false;

  // D6 startup grace — ignore Enter/Exit entirely until UART TX boot flap settles
  if (!enterExitGraceDone) {
    updateButtonPressEdge(enterExitBtn, enterExitHeld);  // keep debouncer ticking
    enterExitHeld = false;
    if (now >= ENTEREXITGRACE_MS) {
      enterExitGraceDone = true;
      // reset debounce state cleanly after grace so whatever the pin
      // is doing right now becomes the new stable baseline
      enterExitBtn.stableState  = HIGH;
      enterExitBtn.lastReading  = HIGH;
      enterExitBtn.lastChangeTime = now;
      Serial.println("D6 grace period done");
    }
  } else {
    updateButtonPressEdge(enterExitBtn, enterExitHeld);
  }

  // Enter/Exit: tap = Enter, hold >= ENTEREXIT_HOLD_MS = Escape (fires once per hold)
  if (enterExitGraceDone && Keyboard.isConnected()) {
    if (enterExitHeld) {
      if (enterExitPressStart == 0) enterExitPressStart = now;  // arm timer on first held sample
      if (!enterExitHoldFired && (now - enterExitPressStart >= ENTEREXIT_HOLD_MS)) {
        Serial.println("Enter/Exit: Escape (hold)");
        Keyboard.press(KEY_ESC);
        delay(20);
        Keyboard.releaseAll();
        enterExitHoldFired = true;
      }
    } else {
      if (enterExitPressStart != 0 && !enterExitHoldFired) {
        // released before hold threshold — it was a tap, send Enter
        Serial.println("Enter/Exit: Enter (tap)");
        Keyboard.press(KEY_RETURN);
        delay(20);
        Keyboard.releaseAll();
      }
      // reset for next press
      enterExitPressStart = 0;
      enterExitHoldFired  = false;
    }
  }

  bool clutchPressEdge = updateButtonPressEdge(clutchBtn, clutchHeldRaw);

  if (clutchPressEdge) {
    clutchActive = !clutchActive;
    Serial.println(clutchActive ? "Clutch: ON" : "Clutch: OFF");
    smoothingInitialized = false;  // avoid a stale-smoothing jump when returning to mouse mode
    leftClutchPressStart  = 0; leftClutchHoldFired  = false;
    rightClutchPressStart = 0; rightClutchHoldFired = false;
    if (clutchActive) {
      // FIX: averaged baseline instead of one raw sample
      getPitchRollAveraged(clutchTiltBaseline, clutchRollBaseline);
      clutchRollArmed = true;
      clutchRollHoldActive = false;
    }
    clutchPressStartTime = now;
    clutchEasterEggArmed = true;
  }

  // Easter egg: hold D7 for 3+ seconds (a normal tap already toggled clutch
  // above — this just adds a bonus action on top if you keep holding)
  if (clutchHeldRaw && clutchEasterEggArmed && (now - clutchPressStartTime > 3000)) {
    sendOpenURL(EASTER_EGG_URL);
    clutchEasterEggArmed = false;  // fire once per hold, re-arms on next press
  }

  // ---------------- CLUTCH MODE ----------------
  if (clutchActive) {
    // tare re-zeros the CLUTCH reference (not cursor home) while clutched —
    // same hold-to-reposition-then-release-to-set pattern as normal mode
    if (tareReleaseEdge) {
      getPitchRollAveraged(clutchTiltBaseline, clutchRollBaseline);  // FIX: averaged
      clutchRollArmed = true;
      clutchRollHoldActive = false;
    }
    if (tareHeld) {
      lastSampleTime = now;
      return;  // freeze zoom/up-down while repositioning, same as normal mode
    }

    // Left button: tap = Undo, hold = Sketch (Shift+S)
    if (leftHeld) {
      if (leftClutchPressStart == 0) leftClutchPressStart = now;
      if (!leftClutchHoldFired && (now - leftClutchPressStart >= CLUTCH_BTN_HOLD_MS)) {
        sendMacro("Sketch (hold)", sketchMacro);
        leftClutchHoldFired = true;
      }
    } else {
      if (leftClutchPressStart != 0 && !leftClutchHoldFired)
        sendMacro("Undo (tap)", undoMacro);
      leftClutchPressStart = 0;
      leftClutchHoldFired  = false;
    }

    // Right button: tap = Redo, hold = Extrude (Shift+E)
    if (rightHeld) {
      if (rightClutchPressStart == 0) rightClutchPressStart = now;
      if (!rightClutchHoldFired && (now - rightClutchPressStart >= CLUTCH_BTN_HOLD_MS)) {
        sendMacro("Extrude (hold)", extrudeMacro);
        rightClutchHoldFired = true;
      }
    } else {
      if (rightClutchPressStart != 0 && !rightClutchHoldFired)
        sendMacro("Redo (tap)", redoMacro);
      rightClutchPressStart = 0;
      rightClutchHoldFired  = false;
    }

    float pitch, roll, magnitude;
    getPitchRoll(pitch, roll, magnitude);

    const float magnitudeTolerance = 0.3f;
    if (fabs(magnitude - 1.0f) < magnitudeTolerance) {  // skip fast-shake garbage
      float deltaTilt = pitch - clutchTiltBaseline;
      float deltaRoll = (roll - clutchRollBaseline) * invertClutchRoll;

      // Only the DOMINANT axis acts this sample — same "which one's bigger"
      // rule that fixed cross-talk during real gesture testing earlier.
      // Without this, a pure up/down tilt also nudges pitch enough to fire
      // continuous zoom at the same time, drowning out the discrete trigger.
      bool leftRightDominant = fabs(deltaTilt) > fabs(deltaRoll);

      // --- left/right tilt: zoom in/out, one tick per interval while tilted ---
      if (leftRightDominant && fabs(deltaTilt) > zoomTiltDeadzoneDeg) {
        if (now - lastZoomTick > zoomTickIntervalMs) {
          lastZoomTick = now;
          if (Keyboard.isConnected()) {
            Mouse.move(0, 0, ((deltaTilt > 0) ? 1 : -1) * invertZoom);  // FIX: invertZoom
          }
        }
      }

      // --- up/down tilt: discrete Zoom-to-fit / View-normal, sustained-hold
      // pattern (like "palm up"), armed/disarmed on return to neutral so it
      // fires once per deliberate tilt, not repeatedly while held ---
      if (!leftRightDominant) {
        if (clutchRollArmed) {
          if (deltaRoll > clutchUpDownThreshold) {
            if (!clutchRollHoldActive || clutchRollPendingDir != 1) {
              clutchRollHoldActive = true;
              clutchRollHoldStart = now;
              clutchRollPendingDir = 1;
            } else if (now - clutchRollHoldStart > clutchUpDownHoldMs) {
              sendMacro("Zoom to fit", zoomFitMacro);
              clutchRollArmed = false;
              clutchRollHoldActive = false;
            }
          } else if (deltaRoll < -clutchUpDownThreshold) {
            if (!clutchRollHoldActive || clutchRollPendingDir != -1) {
              clutchRollHoldActive = true;
              clutchRollHoldStart = now;
              clutchRollPendingDir = -1;
            } else if (now - clutchRollHoldStart > clutchUpDownHoldMs) {
              sendMacro("View normal", viewNormalMacro);
              clutchRollArmed = false;
              clutchRollHoldActive = false;
            }
          } else {
            clutchRollHoldActive = false;
          }
        } else if (fabs(deltaRoll) < clutchRollRearmDeg) {
          // FIX(hysteresis): re-arm only once the hand is genuinely back
          // near neutral — BELOW the fire threshold (7.5° < 15°), not at
          // 20° like before. Fire high, re-arm low: holding a steady tilt
          // anywhere past the fire level now fires exactly once.
          clutchRollArmed = true;
        }
      }
    }

    lastSampleTime = now;
    return;
  }

  // ---------------- NORMAL MODE: mouse clicks + tilt tracking ----------------
  if (Keyboard.isConnected()) {
    if (leftHeld && !leftPressed)   { Mouse.press(MOUSE_LEFT);  leftPressed = true; }
    if (!leftHeld && leftPressed)   { Mouse.release(MOUSE_LEFT); leftPressed = false; }
    if (rightHeld && !rightPressed) { Mouse.press(MOUSE_RIGHT);  rightPressed = true; }
    if (!rightHeld && rightPressed) { Mouse.release(MOUSE_RIGHT); rightPressed = false; }
  }

  if (tareReleaseEdge) {
    getPitchRollAveraged(pitchOffset, rollOffset);  // FIX: averaged
    smoothingInitialized = false;
  }

  if (tareHeld) {
    lastSampleTime = now;
    return;
  }

  if (now - lastSampleTime < sampleMs) return;
  lastSampleTime = now;

  float pitch, roll, magnitude;
  getPitchRoll(pitch, roll, magnitude);

  const float magnitudeTolerance = 0.3f;
  if (fabs(magnitude - 1.0f) >= magnitudeTolerance) return;  // fast-shake rejection

  if (!smoothingInitialized) {
    smoothedPitch = pitch;
    smoothedRoll  = roll;
    smoothingInitialized = true;
  } else {
    smoothedPitch = smoothingAlpha * pitch + (1 - smoothingAlpha) * smoothedPitch;
    smoothedRoll  = smoothingAlpha * roll  + (1 - smoothingAlpha) * smoothedRoll;
  }

  float deltaPitch = smoothedPitch - pitchOffset;
  float deltaRoll  = smoothedRoll  - rollOffset;
  if (fabs(deltaPitch) < deadzoneDeg) deltaPitch = 0;
  if (fabs(deltaRoll)  < deadzoneDeg) deltaRoll  = 0;

  float speedX = deltaPitch * sensitivity * invertX;
  float speedY = deltaRoll  * sensitivity * invertY;
  speedX = constrain(speedX, -maxCursorSpeed, maxCursorSpeed);
  speedY = constrain(speedY, -maxCursorSpeed, maxCursorSpeed);

  xRemainder += speedX;
  yRemainder += speedY;
  int moveX = (int)xRemainder;
  int moveY = (int)yRemainder;
  xRemainder -= moveX;
  yRemainder -= moveY;

  if ((moveX != 0 || moveY != 0) && Keyboard.isConnected()) {
    Mouse.move(moveX, moveY);
  }
}