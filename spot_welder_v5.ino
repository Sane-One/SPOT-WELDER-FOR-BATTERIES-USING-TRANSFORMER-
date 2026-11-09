// =====================================================================
//  SPOT WELDER CONTROLLER v3  (Arduino Nano / ATmega328P)
//  Transformer + TRIAC + MOC3250 + H11AA1 zero-cross
//  KY-040 encoder (pin-change interrupt), SSD1306 OLED,
//  10k NTC on A1, one status LED (steady = power on, fast blink = welding soon)
//
//  Pin map
//    D2  H11AA1 output (zero-cross, needs 10k pull-up to 5V)
//    D4  limit switch / pedal to GND
//    D5  status LED (via 330 ohm to GND)
//    D8  MOC3250 LED (via resistor)
//    D9  indicator LED
//    A0  KY-040 CLK   (PC0)   <-- must stay on A0 / A3 because the
//    A3  KY-040 DT    (PC3)       encoder ISR reads PORTC directly
//    A2  KY-040 SW
//    A1  NTC divider: 5V -> 10k -> A1 -> NTC -> GND
//    A4  OLED SDA, A5 OLED SCL (0x3C)
// =====================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/pgmspace.h>
#include <math.h>

Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// ---------------- PINS ----------------
const byte ZERO_PIN  = 2;
const byte LIMIT_PIN = 4;
const byte STATUS_LED_PIN = 5;  // steady = power on, fast blink = about to weld
const byte MOC_PIN   = 8;
const byte LED_PIN   = 9;

const byte ENC_CLK = A0;   // PC0
const byte NTC_PIN = A1;
const byte ENC_SW  = A2;
const byte ENC_DT  = A3;   // PC3

// ---------------- SETTINGS ----------------
const unsigned long HOLD_MS     = 3000;
const unsigned long DEBOUNCE_MS = 30;

const int MIN_PULSE_MS  = 20;
const int MAX_PULSE_MS  = 100;
const int PULSE_STEP_MS = 20;      // must be a multiple of 10 (50 Hz half-cycle)

// Encoder: signal transitions per click. Most KY-040 modules give 4.
// If you must turn two clicks for one step, set this to 2.
// If one click jumps two steps, set it to 8.
#define ENC_TRANSITIONS_PER_STEP 4

// Zero-cross signal sanity (H11AA1 gives a pulse every half-cycle, ~10 ms at 50 Hz)
const unsigned long MIN_INTERVAL_US = 5000;    // shorter = glitch, ignored
const unsigned long MAX_INTERVAL_US = 15000;   // longer = a crossing was missed

// Thermistor (10k NTC, 10k series resistor, NTC on the GND side)
const float SERIES_R  = 10000.0;
const float NTC_R0    = 10000.0;
const float NTC_T0_K  = 298.15;
const float NTC_BETA  = 3950.0;    // CHECK your thermistor's datasheet value

// Thermal protection
const float LOCKOUT_C  = 80.0;     // welding blocked above this

// Status LED blink toggle time while holding / welding (smaller = faster)
const unsigned long BLINK_MS = 100;

// ---------------- STATES ----------------
enum State { READY, HOLDING, WAIT_ZERO, PULSING, DONE, NO_ZERO, OVERTEMP };
State state = READY;

// ---------------- ZERO-CROSS / PULSE (shared with ISR) ----------------
volatile unsigned long previousZeroUs = 0;
volatile unsigned long lastZeroUs = 0;
volatile unsigned int  zeroCount = 0;
volatile byte          validZeroCount = 0;

volatile bool armed = false;           // loop sets, ISR consumes at next crossing
volatile bool pulseActive = false;
volatile bool pulseStarted = false;
volatile bool pulseFinished = false;
volatile bool pulseFault = false;
volatile byte pulseHalfCycles = 2;
volatile unsigned int pulseEndZeroCount = 0;

// ---------------- ENCODER (shared with ISR) ----------------
volatile byte   encPrev = 0;
volatile int8_t encAcc = 0;
volatile int8_t encSteps = 0;

// Index = (previous << 2) | current, state = (CLK << 1) | DT
// Clockwise: 3 -> 1 -> 0 -> 2 -> 3.  Invalid (bounce) transitions = 0.
static const int8_t ENC_TABLE[16] PROGMEM = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ---------------- OTHER VARIABLES ----------------
int pulseTimeMs = MIN_PULSE_MS;

bool rawPressed = false, stablePressed = false;
bool oledOK = false;

unsigned long changedAt = 0, holdStart = 0, waitStart = 0;
unsigned long ledStart = 0, displayTime = 0, pulseStartUs = 0;

bool rawEncBtn = false, stableEncBtn = false;
unsigned long encBtnChangedAt = 0;

float tempC = 25.0;
bool  tempOK = false;
unsigned long tempTime = 0;

// what the OLED currently shows (redraw only on change)
State shownState = READY;
int   shownPulse = -1;
int   shownTempX10 = -9999;
bool  shownOk = false;

// ---------------- HELPERS ----------------
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void stopOutput() {
  noInterrupts();
  armed = false;
  pulseActive = false;
  digitalWrite(MOC_PIN, LOW);
  interrupts();
}

// ---------------- ZERO-CROSS ISR (D2, rising edge) ----------------
void zeroCrossISR() {
  unsigned long t = micros();
  unsigned long interval = t - previousZeroUs;

  // Glitch / bounce: ignore completely
  if (interval < MIN_INTERVAL_US) return;
  previousZeroUs = t;

  if (interval <= MAX_INTERVAL_US) {
    if (validZeroCount < 10) validZeroCount++;
  } else {
    validZeroCount = 0;
    // A crossing was missed while firing: shut off immediately
    if (pulseActive) {
      digitalWrite(MOC_PIN, LOW);
      pulseActive = false;
      armed = false;
      pulseFault = true;
    }
  }

  if (validZeroCount >= 4) {
    lastZeroUs = t;
    zeroCount++;

    if (pulseActive) {
      if (zeroCount == pulseEndZeroCount) {
        digitalWrite(MOC_PIN, LOW);
        pulseActive = false;
        pulseFinished = true;
      }
    } else if (armed) {
      // Start exactly at the zero crossing (no loop() latency)
      armed = false;
      pulseEndZeroCount = zeroCount + pulseHalfCycles;
      pulseActive = true;
      pulseStarted = true;
      digitalWrite(MOC_PIN, HIGH);
    }
  }
}

// ---------------- ENCODER ISR (A0 / A3 pin change) ----------------
ISR(PCINT1_vect) {
  byte pins = PINC;
  byte cur = ((pins & _BV(PC0)) ? 2 : 0) | ((pins & _BV(PC3)) ? 1 : 0);

  encAcc += (int8_t)pgm_read_byte(&ENC_TABLE[(encPrev << 2) | cur]);
  encPrev = cur;

  if (encAcc >= ENC_TRANSITIONS_PER_STEP)       { encAcc = 0; encSteps++; }
  else if (encAcc <= -ENC_TRANSITIONS_PER_STEP) { encAcc = 0; encSteps--; }
}

void handleEncoder(unsigned long now) {
  int8_t steps;
  noInterrupts();
  steps = encSteps;
  encSteps = 0;
  interrupts();

  if (state == READY && steps != 0) {
    int newMs = pulseTimeMs + steps * PULSE_STEP_MS;
    if (newMs < MIN_PULSE_MS) newMs = MIN_PULSE_MS;
    if (newMs > MAX_PULSE_MS) newMs = MAX_PULSE_MS;
    if (newMs != pulseTimeMs) {
      pulseTimeMs = newMs;
      Serial.print(F("Pulse time: "));
      Serial.print(pulseTimeMs);
      Serial.println(F(" ms"));
    }
  }

  // Encoder push button: reset pulse to minimum (READY only)
  bool reading = digitalRead(ENC_SW) == LOW;
  if (reading != rawEncBtn) {
    rawEncBtn = reading;
    encBtnChangedAt = now;
  }
  if (now - encBtnChangedAt >= DEBOUNCE_MS && reading != stableEncBtn) {
    stableEncBtn = reading;
    if (stableEncBtn && state == READY) {
      pulseTimeMs = MIN_PULSE_MS;
      Serial.println(F("Pulse reset to 20ms"));
    }
  }
}

// ---------------- THERMISTOR ----------------
float readTempC(bool &ok) {
  long sum = 0;
  for (byte i = 0; i < 8; i++) sum += analogRead(NTC_PIN);
  float adc = sum / 8.0;

  if (adc < 10.0 || adc > 1013.0) {   // open or shorted sensor
    ok = false;
    return 0;
  }
  float r = SERIES_R * adc / (1023.0 - adc);
  float k = 1.0 / (1.0 / NTC_T0_K + log(r / NTC_R0) / NTC_BETA);
  ok = true;
  return k - 273.15;
}

void updateThermal(unsigned long now) {
  if (state == WAIT_ZERO || state == PULSING) return;
  if (now - tempTime < 500) return;
  tempTime = now;

  tempC = readTempC(tempOK);
}

// ---------------- OLED ----------------
void drawDisplay() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  oled.println(F("SPOT WELDER"));

  oled.setCursor(0, 12);
  switch (state) {
    case READY:    oled.println(F("READY"));          break;
    case HOLDING:  oled.println(F("HOLD 3 SECONDS")); break;
    case DONE:     oled.println(F("PULSE COMPLETE")); break;
    case NO_ZERO:  oled.println(F("NO ZERO SIGNAL")); break;
    case OVERTEMP: oled.println(F("TOO HOT - WAIT")); break;
    default: break;
  }

  oled.setCursor(0, 26);
  oled.print(F("PULSE: "));
  oled.print(pulseTimeMs);
  oled.println(F(" ms"));

  oled.setCursor(0, 38);
  oled.print(F("TEMP: "));
  if (tempOK) {
    oled.print(tempC, 1);
    oled.print(F(" C"));
  } else {
    oled.print(F("NTC ERR"));
  }

  oled.setCursor(0, 52);
  oled.println(F("Release to reset"));

  oled.display();
}

// ---------------- SETUP ----------------
void setup() {
  pinMode(MOC_PIN, OUTPUT);
  digitalWrite(MOC_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);   // steady ON = power on

  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(ZERO_PIN, INPUT);          // external 10k pull-up
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  Serial.begin(115200);

  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Serial.print(F("OLED: "));
  Serial.println(oledOK ? F("OK") : F("FAILED (check wiring / RAM)"));
  Serial.print(F("Free RAM: "));
  Serial.print(freeRam());
  Serial.println(F(" bytes"));

  // If the pedal is already pressed at power-up, require a release first
  rawPressed = stablePressed = (digitalRead(LIMIT_PIN) == LOW);

  tempC = readTempC(tempOK);

  // Encoder pin-change interrupt on A0 (PCINT8) and A3 (PCINT11)
  byte pins = PINC;
  encPrev = ((pins & _BV(PC0)) ? 2 : 0) | ((pins & _BV(PC3)) ? 1 : 0);
  PCMSK1 |= _BV(PCINT8) | _BV(PCINT11);
  PCIFR  |= _BV(PCIF1);
  PCICR  |= _BV(PCIE1);

  attachInterrupt(digitalPinToInterrupt(ZERO_PIN), zeroCrossISR, RISING);

  Serial.println(F("SPOT WELDER READY"));
}

// ---------------- MAIN LOOP ----------------
void loop() {
  unsigned long now = millis();

  handleEncoder(now);
  updateThermal(now);

  // 1. Limit switch debounce
  bool reading = digitalRead(LIMIT_PIN) == LOW;
  if (reading != rawPressed) {
    rawPressed = reading;
    changedAt = now;
  }
  if (now - changedAt >= DEBOUNCE_MS && reading != stablePressed) {
    stablePressed = reading;
    if (stablePressed) {
      state = HOLDING;
      holdStart = now;
      Serial.println(F("HOLDING..."));
    } else {
      stopOutput();            // release = stop + reset
      state = READY;
      Serial.println(F("RESET - READY"));
    }
  }

  // 2. Hold complete -> check temperature and zero-cross, then arm
  if (state == HOLDING && now - holdStart >= HOLD_MS) {
    if (tempOK && tempC >= LOCKOUT_C) {
      state = OVERTEMP;
      Serial.println(F("ERROR: TOO HOT"));
    } else {
      unsigned long last;
      byte count;
      noInterrupts();
      last = lastZeroUs;
      count = validZeroCount;
      interrupts();

      if (count >= 4 && (unsigned long)(micros() - last) < 20000UL) {
        noInterrupts();
        pulseHalfCycles = pulseTimeMs / 10;   // 50 Hz: 10 ms per half-cycle
        pulseStarted = false;
        pulseFinished = false;
        pulseFault = false;
        armed = true;                         // ISR fires at next crossing
        interrupts();

        state = WAIT_ZERO;
        waitStart = now;
        Serial.println(F("Waiting for zero crossing"));
      } else {
        state = NO_ZERO;
        Serial.println(F("ERROR: NO ZERO CROSS"));
      }
    }
  }

  // 3. Wait for the ISR to start the pulse
  if (state == WAIT_ZERO) {
    bool started;
    noInterrupts();
    started = pulseStarted;
    interrupts();

    if (started) {
      pulseStartUs = micros();
      digitalWrite(LED_PIN, HIGH);
      ledStart = millis();
      state = PULSING;
      Serial.print(F("PULSE START: "));
      Serial.print(pulseTimeMs);
      Serial.println(F(" ms"));
    } else if (now - waitStart > 100) {
      stopOutput();
      state = NO_ZERO;
      Serial.println(F("ERROR: ZERO CROSS TIMEOUT"));
    }
  }

  // 4. Pulse supervision (ISR ends the pulse; this only watches for faults)
  if (state == PULSING) {
    bool finished, fault;
    unsigned long last;
    noInterrupts();
    finished = pulseFinished;
    fault = pulseFault;
    last = lastZeroUs;
    interrupts();

    if (fault) {
      stopOutput();
      state = NO_ZERO;
      Serial.println(F("ERROR: ZERO SIGNAL LOST"));
    } else if (finished) {
      state = DONE;
      Serial.println(F("PULSE COMPLETE"));
    } else if ((unsigned long)(micros() - last) > 25000UL ||
               (unsigned long)(micros() - pulseStartUs) >
                 (unsigned long)pulseTimeMs * 1000UL + 20000UL) {
      stopOutput();
      state = NO_ZERO;
      Serial.println(F("ERROR: PULSE WATCHDOG"));
    }
  }

  // 5. LED indicator stays on 500 ms after a pulse
  if (state != PULSING && millis() - ledStart >= 500) {
    digitalWrite(LED_PIN, LOW);
  }

  // 5b. Status LED: steady ON normally; blinks fast from pedal press until the
  //     weld pulse ends, then returns to steady ON
  if (state == HOLDING || state == WAIT_ZERO || state == PULSING) {
    digitalWrite(STATUS_LED_PIN, ((now / BLINK_MS) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }

  // 6. OLED: only redraw when something changed, never during a pulse
  if (oledOK && state != WAIT_ZERO && state != PULSING &&
      millis() - displayTime >= 100) {

    int tx10 = tempOK ? (int)(tempC * 10.0) : -9999;
    if (state != shownState || pulseTimeMs != shownPulse ||
        tx10 != shownTempX10 || tempOK != shownOk) {

      shownState = state;
      shownPulse = pulseTimeMs;
      shownTempX10 = tx10;
      shownOk = tempOK;

      displayTime = millis();
      drawDisplay();
    }
  }
}
