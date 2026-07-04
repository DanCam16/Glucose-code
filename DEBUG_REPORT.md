# Firmware Debug Report - Glucose Monitor v2.0

## Critical Issues Found

### 1. **CRITICAL: ADC Reference Conflict** ⚠️
**Location:** Lines 321-328 (measureIntensity) & Line 754 (configureADC)

**Problem:**
- `configureADC()` sets `analogReference(INTERNAL)` (1.1V reference)
- `measureIntensity()` assumes photodiode is connected to AVCC rail
- **Result:** Photodiode measurements will be wildly incorrect (>1000% error possible)

**Fix:**
```cpp
// OPTION A: Use EXTERNAL reference with proper capacitor
analogReference(EXTERNAL);  // Requires 100nF capacitor on AREF pin

// OPTION B: Use DEFAULT (AVCC) for photodiode, INTERNAL only for battery
// Requires separate reference switching or dual measurement strategy
```

**Recommendation:** Use EXTERNAL reference with 100nF filtering capacitor on AREF pin for best accuracy on photodiode circuit.

---

### 2. **CRITICAL: Variable Length Arrays (VLA) Not Supported**
**Location:** Line 373 in medianFilter()

**Problem:**
```cpp
uint16_t sorted[size];  // VLA - NOT supported in standard C++ on Arduino
```

Arduino's avr-gcc does support VLAs, but it's non-standard and uses precious stack memory. With FILTER_SIZE=5, this creates 5×2=10 bytes on stack per call, causing potential stack overflow.

**Fix:**
```cpp
uint16_t sorted[FILTER_SIZE];  // Use fixed size from #define
```

---

### 3. **CRITICAL: Infinite Loop on OLED Failure**
**Location:** Lines 228-231 in setup()

**Problem:**
```cpp
if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
  Serial.println(F("OLED initialization failed!"));
  while (1);  // Infinite loop - device hangs
}
```

If I2C communication fails, device is permanently frozen. No way to recover or continue.

**Fix:**
```cpp
if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
  Serial.println(F("OLED initialization failed!"));
  // Continue anyway - OLED not critical for functionality
  // Could add watchdog timer for reset if desired
}
```

---

### 4. **CRITICAL: LED PWM Not Set**
**Location:** Lines 222-223 in setup()

**Problem:**
```cpp
pinMode(PIN_LED, OUTPUT);
digitalWrite(PIN_LED, LOW);
// Missing: analogWrite(PIN_LED, LED_PWM_VALUE);
```

LED is configured as digital output, NOT PWM. The `digitalWrite()` only allows ON/OFF. **No current limiting—LED may burn out!**

**Fix:**
```cpp
pinMode(PIN_LED, OUTPUT);
digitalWrite(PIN_LED, LOW);
// Add PWM setup
analogWrite(PIN_LED, LED_PWM_VALUE);  // Set PWM at startup
```

Better yet, use PWM in measuring function only:
```cpp
void measureIntensity() {
  analogWrite(PIN_LED, LED_PWM_VALUE);  // PWM ON
  delayMicroseconds(LED_ON_DELAY_MS * 1000);
  uint16_t i_led_on = readADCAverage(ADC_PHOTODIODE, 4);
  digitalWrite(PIN_LED, LOW);  // Digital OFF
  // ...
}
```

---

### 5. **BUG: Incorrect ADC Channel Selection**
**Location:** Line 353 in readADCAverage()

**Problem:**
```cpp
sum += analogRead(channel);  // channel = 0 or 1 (correct for A0/A1)
```

This works, BUT the code uses `#define` pins like `PIN_PHOTODIODE A0` which is actually 14 (on Nano). Should be:
```cpp
#define ADC_PHOTODIODE 0   // Correct - maps to A0
#define ADC_BATTERY 1      // Correct - maps to A1
```

**Status:** Actually correct, but confusing. Add comment clarifying that `A0=0`, `A1=1` in #define section.

---

### 6. **BUG: Ambient Light Subtraction Underflow Risk**
**Location:** Lines 335-339 in measureIntensity()

**Problem:**
```cpp
if (i_led_on <= i_led_off) {
  return 1;  // Safety clamp
}
```

If ambient light is very bright, `i_led_off` might exceed `i_led_on`. This is physically possible if:
- LED current is low
- Ambient light is very bright
- Photodiode response is nonlinear

**Symptom:** Device shows zero intensity when it should show negative absorbance (invalid).

**Fix:**
```cpp
// Better error handling
if (i_led_on <= i_led_off) {
  Serial.println(F("WARNING: Ambient light > LED signal!"));
  Serial.print(F("LED ON: ")); Serial.print(i_led_on);
  Serial.print(F(", Ambient: ")); Serial.println(i_led_off);
  return 1;  // Return minimum intensity, display warning
}
```

---

### 7. **BUG: EMA Initialization Race Condition**
**Location:** Lines 248-250 in setup()

**Problem:**
```cpp
delay(100);
current_intensity = measureIntensity();
ema_intensity = current_intensity;  // Assigned as float from uint16_t
```

If `measureIntensity()` is called again immediately in loop (before 10ms has passed), `ema_intensity` might be uninitialized or stale.

**Fix:**
Initialize properly:
```cpp
current_intensity = measureIntensity();
ema_intensity = (float)current_intensity;
// In loop, add bounds check
if (ema_intensity == 0) {
  ema_intensity = (float)current_intensity;
}
```

---

### 8. **BUG: Calibration Button State Machine**
**Location:** Lines 531-550 in handleCalibrationButton()

**Problem:**
```cpp
static uint32_t cal_hold_start = 0;

if (digitalRead(PIN_CAL_BUTTON) == LOW) {
  if (cal_hold_start == 0) {
    cal_hold_start = millis();
  }
  if (millis() - cal_hold_start >= CAL_BUTTON_HOLD_MS) {
    performCalibration();
    cal_hold_start = 0;  // Reset
  }
} else {
  cal_hold_start = 0;  // Button released
}
```

**Issues:**
1. **Double-trigger:** `performCalibration()` may be called multiple times if button held > 2s
2. **No debouncing:** Electrical noise might trigger false presses
3. **Blocking behavior:** `performCalibration()` calls `delay()`, freezing measurement loop

**Fix:**
```cpp
void handleCalibrationButton() {
  static uint32_t cal_hold_start = 0;
  static bool calibration_in_progress = false;
  const uint16_t DEBOUNCE_MS = 20;
  static uint32_t last_button_read = 0;
  
  // Debounce: only read button every 20ms
  if (millis() - last_button_read < DEBOUNCE_MS) {
    return;
  }
  last_button_read = millis();
  
  if (digitalRead(PIN_CAL_BUTTON) == LOW) {
    if (cal_hold_start == 0) {
      cal_hold_start = millis();
    }
    
    // Trigger only ONCE when threshold crossed
    if (!calibration_in_progress && 
        millis() - cal_hold_start >= CAL_BUTTON_HOLD_MS) {
      calibration_in_progress = true;
      performCalibration();
      cal_hold_start = 0;
    }
  } else {
    // Button released
    cal_hold_start = 0;
    calibration_in_progress = false;
  }
}
```

---

### 9. **PERFORMANCE BUG: Blocking Delays in Calibration**
**Location:** Lines 563-622 in performCalibration()

**Problem:**
```cpp
for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
  digitalWrite(PIN_LED, HIGH);
  delayMicroseconds(LED_ON_DELAY_MS * 1000);
  // ...
  delay(50);  // <-- BLOCKING for 50ms × 20 samples = 1000ms total
}
```

Device is frozen for 1+ second during calibration. OLED won't update, measurements freeze, all I/O is blocked.

**Fix:**
Reduce to 10 samples with shorter delays:
```cpp
const uint8_t CAL_SAMPLES = 10;  // 500ms instead of 1000ms
for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
  digitalWrite(PIN_LED, HIGH);
  delayMicroseconds(LED_ON_DELAY_MS * 1000);
  uint16_t intensity = readADCAverage(ADC_PHOTODIODE, 4);
  digitalWrite(PIN_LED, LOW);
  delay(25);  // Reduce from 50ms to 25ms
}
```

---

### 10. **BUG: ADC Configuration Overwrites Before Use**
**Location:** Lines 751-762 in configureADC()

**Problem:**
```cpp
void configureADC() {
  analogReference(INTERNAL);  // Sets reference
  // ADC prescaler setup...
  ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
```

The first line `analogReference(INTERNAL)` should happen BEFORE the first `analogRead()` call, but we already called `measureIntensity()` in setup() at line 249!

**Fix:**
Move `configureADC()` before first measurement:
```cpp
void setup() {
  // ... pin setup ...
  
  // Configure ADC BEFORE any measurements
  configureADC();
  
  // Initialize OLED ...
  
  // Load calibration ...
  
  // NOW measure
  delay(100);
  current_intensity = measureIntensity();
  ema_intensity = current_intensity;
}
```

---

### 11. **PORTABILITY BUG: Pin Definitions**
**Location:** Lines 132-139

**Problem:**
```cpp
#define PIN_PHOTODIODE A0
#define PIN_BATTERY A1
#define PIN_LED D3
#define PIN_CAL_BUTTON D4
```

`D3` and `D4` are not valid Arduino constants. They only work on some boards. Should use numeric pin numbers:

**Fix:**
```cpp
#define PIN_PHOTODIODE A0      // ADC channel 0
#define PIN_BATTERY A1         // ADC channel 1
#define PIN_LED 3              // GPIO pin 3 (PWM capable)
#define PIN_CAL_BUTTON 4       // GPIO pin 4
```

---

## Summary of Severity Levels

| Issue | Severity | Impact |
|-------|----------|--------|
| ADC Reference Conflict | CRITICAL 🔴 | 1000%+ measurement error |
| LED PWM Not Set | CRITICAL 🔴 | LED burnout, wrong measurements |
| VLA Stack Overflow | HIGH 🟠 | Unpredictable crashes |
| OLED Infinite Loop | HIGH 🟠 | Device hangs permanently |
| Ambient Light Underflow | HIGH 🟠 | Negative intensity, crash |
| Button Double-Trigger | MEDIUM 🟡 | False calibrations |
| ADC Config Order | MEDIUM 🟡 | Wrong reference on first read |
| Pin Definition | MEDIUM 🟡 | Portability issues |
| Calibration Blocking | MEDIUM 🟡 | UI freezes |
| EMA Race Condition | LOW 🟢 | Edge case errors |

---

## Testing Recommendations

1. **Compile and upload** the fixed version
2. **Serial monitor test:**
   - Watch for "ADC configured..." message
   - Watch for "No valid calibration..." message
3. **Hardware test:**
   - Connect multimeter to LED pin - should see ~78% PWM
   - Cover photodiode - intensity should drop to near-zero
   - Remove cover - intensity should increase
4. **Calibration test:**
   - Press CAL button for 2+ seconds
   - Device should measure and store baseline
   - OLED should show "CALIBRATION SUCCESS"
5. **Measurement test:**
   - Place finger on sensor
   - OLED should show increasing intensity
   - After stabilization, glucose value should appear

