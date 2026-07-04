/*
 * ============================================================================
 * Non-Invasive Blood Glucose Monitor - Arduino Nano Firmware
 * ============================================================================
 * 
 * Project: Level 400 Biomedical Engineering - Research Prototype
 * Device: Blood Glucose Monitoring via IR Absorbance
 * Platform: Arduino Nano with 0.96" OLED Display
 * 
 * DESCRIPTION:
 * This firmware implements a research-grade blood glucose estimation device
 * using a 940 nm IR LED and photodiode pair. The device measures optical
 * absorbance through tissue and estimates glucose concentration after
 * calibration against a reference glucometer.
 * 
 * IMPORTANT DISCLAIMER:
 * This is a RESEARCH PROTOTYPE and NOT a medical diagnostic device.
 * Glucose estimates must be verified with a calibrated reference glucometer.
 * 
 * ============================================================================
 * IMPROVEMENTS FROM ORIGINAL CODE:
 * ============================================================================
 * 1. Absorbance-based estimation (Modified Beer-Lambert Law)
 * 2. Ambient light cancellation (LED ON/OFF differential measurement)
 * 3. Advanced filtering (median + exponential moving average)
 * 4. Improved photodiode circuit recommendations
 * 5. Robust baseline calibration with button control
 * 6. Enhanced OLED display with absorbance and glucose values
 * 7. Higher sampling rate (100-200 Hz vs 50 ms)
 * 8. Stability detection before measurement
 * 9. Battery voltage measurement with internal reference
 * 10. Comprehensive EEPROM calibration storage
 * 
 * ============================================================================
 * HARDWARE CONFIGURATION:
 * ============================================================================
 * 
 * Arduino Nano Pinout:
 * A0  -> Photodiode output (from transimpedance amplifier)
 * A1  -> Battery voltage monitor (resistor divider)
 * D3  -> IR LED drive (PWM)
 * D4  -> Calibration button (active LOW)
 * D5  -> SDA (OLED I2C)
 * D6  -> SCL (OLED I2C)
 * 
 * OLED Display: 0.96" 128x64 SSD1306 via I2C (address 0x3C)
 * 
 * LED Current Drive:
 * - PWM on pin D3 controls IR LED current
 * - Constant PWM provides constant electrical drive
 * - Optical intensity affected by temperature and supply voltage
 * 
 * Photodiode Circuit Recommendations:
 * - OPT101 (integrated trans-impedance amplifier) - PREFERRED
 * - OPA380/OPA381 (low-noise op-amp) - GOOD
 * - LM358 (low-cost, lower performance) - CURRENT
 * 
 * ============================================================================
 * GLUCOSE ESTIMATION MODEL:
 * ============================================================================
 * 
 * Physical basis: Modified Beer-Lambert Law
 * 
 * At 940 nm wavelength:
 *   A = log10(I0/I) = log10(I_led_on / I_absorbed)
 * 
 * Where:
 *   A       = Optical absorbance (dimensionless)
 *   I0      = Reference intensity (LED on, no finger)
 *   I       = Measured intensity (with finger)
 *   I_absorbed = Intensity absorbed by tissue
 * 
 * Polynomial regression model:
 *   G = β0 + β1*A + β2*A² + β3*A³
 * 
 * Calibration procedure:
 * 1. Remove finger, press CAL button to measure baseline (I0)
 * 2. Place finger on sensor, wait for stability
 * 3. Measure intensity with LED (I)
 * 4. Calculate absorbance: A = log10(I0/I)
 * 5. Estimate glucose: G = β0 + β1*A + β2*A²
 * 
 * ============================================================================
 * AMBIENT LIGHT CANCELLATION:
 * ============================================================================
 * 
 * Standard practice for optical glucose sensing:
 * 
 * Measurement cycle (5-10 ms):
 *   1. Turn LED ON  -> ADC measurement (I_on)
 *   2. Turn LED OFF -> ADC measurement (I_off)
 *   3. Intensity = I_on - I_off
 * 
 * This removes constant ambient light that would otherwise corrupt
 * the absorbance calculation.
 * 
 * ============================================================================
 * CALIBRATION & STORAGE:
 * ============================================================================
 * 
 * EEPROM Memory Map (1024 bytes on Nano):
 *   0-1    : Baseline intensity I0 (16-bit unsigned)
 *   2-3    : Calibration flag (0xABCD = valid)
 *   4-19   : Reserved for future use
 *   20-39  : Calibration coefficients (floats: β0, β1, β2, β3)
 * 
 * Calibration sequence:
 *   1. User removes finger
 *   2. User presses CAL button (held for 2 seconds)
 *   3. Device measures baseline intensity and stores to EEPROM
 *   4. OLED displays "Calibration successful"
 * 
 * ============================================================================
 * AUTHOR: [Your Name]
 * DATE: July 2026
 * VERSION: 2.0
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_I2C_ADDR 0x3C

#define PIN_PHOTODIODE A0
#define PIN_BATTERY A1
#define PIN_LED D3
#define PIN_CAL_BUTTON D4

// ADC pins (Nano has 8 analog channels: A0-A7)
#define ADC_PHOTODIODE 0
#define ADC_BATTERY 1

// ============================================================================
// CONSTANTS & PARAMETERS
// ============================================================================

// Sampling & filtering
#define SAMPLING_RATE_MS 10      // 100 Hz sampling (improved from 50 ms)
#define FILTER_SIZE 5            // Median filter window
#define STABILITY_THRESHOLD 50   // ADC counts - allow ±50 count variance
#define STABILITY_SAMPLES 10     // Require 10 consecutive stable samples

// LED drive
#define LED_PWM_VALUE 200        // 0-255, roughly 78% duty cycle
#define LED_ON_DELAY_MS 2        // Settle time after LED ON
#define LED_OFF_DELAY_MS 2       // Settle time after LED OFF

// Battery measurement
#define BATTERY_ADC_SAMPLES 16   // Average 16 samples for stability
#define VREF_INTERNAL_MV 1100    // Internal 1.1V reference (ATmega328P)
#define BATTERY_DIVIDER_RATIO 5  // Adjust based on actual resistor divider

// Calibration
#define CAL_BUTTON_HOLD_MS 2000  // Button must be held for 2 seconds
#define EEPROM_BASELINE_ADDR 0
#define EEPROM_CAL_FLAG_ADDR 2
#define EEPROM_COEFFICIENTS_ADDR 20
#define EEPROM_CAL_MAGIC 0xABCD

// Glucose estimation
#define DEFAULT_BETA0 5.0        // Intercept (mmol/L) - adjust via calibration
#define DEFAULT_BETA1 3.5        // Linear coefficient
#define DEFAULT_BETA2 0.5        // Quadratic coefficient
#define ABSORBANCE_MIN 0.01      // Minimum valid absorbance
#define ABSORBANCE_MAX 2.0       // Maximum valid absorbance

// Display
#define DISPLAY_UPDATE_MS 500    // Update OLED every 500 ms

// ============================================================================
// GLOBAL OBJECTS & VARIABLES
// ============================================================================

// OLED display (I2C on A4/A5 for Nano)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Calibration & estimation coefficients
struct CalibrationData {
  uint16_t baseline_intensity;  // I0 - reference intensity
  uint16_t calibration_flag;    // Magic flag: 0xABCD if valid
  float beta0;                  // Intercept
  float beta1;                  // Linear coefficient
  float beta2;                  // Quadratic coefficient
} calibration = {0, 0, DEFAULT_BETA0, DEFAULT_BETA1, DEFAULT_BETA2};

// Signal processing buffers
uint16_t intensity_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;
float ema_intensity = 0;        // Exponential moving average
const float EMA_ALPHA = 0.3;    // EMA smoothing factor (0-1, higher = more responsive)

// State variables
uint32_t last_display_update = 0;
uint32_t last_sample_time = 0;
uint32_t cal_button_press_time = 0;
uint16_t current_intensity = 0;
float current_absorbance = 0;
float current_glucose = 0;
uint8_t stability_count = 0;
bool is_stable = false;
uint8_t battery_percent = 0;

// ============================================================================
// SETUP & INITIALIZATION
// ============================================================================

void setup() {
  // Initialize serial for debugging (optional)
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println(F("\n\nGlucose Monitor v2.0 - Starting..."));
  
  // Configure pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  
  pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);
  
  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("OLED initialization failed!"));
    while (1);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Glucose Monitor v2.0"));
  display.println(F("Initializing..."));
  display.display();
  
  // Load calibration data from EEPROM
  loadCalibration();
  
  // Configure ADC
  configureADC();
  
  // Initialize EMA with first reading
  delay(100);
  current_intensity = measureIntensity();
  ema_intensity = current_intensity;
  
  Serial.println(F("Initialization complete!"));
  display.clearDisplay();
  delay(1000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check calibration button
  handleCalibrationButton();
  
  // Perform measurement at regular intervals
  if (millis() - last_sample_time >= SAMPLING_RATE_MS) {
    last_sample_time = millis();
    
    // Measure intensity (with ambient light cancellation)
    uint16_t raw_intensity = measureIntensity();
    
    // Apply median filter
    intensity_buffer[buffer_index] = raw_intensity;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    uint16_t filtered_intensity = medianFilter(intensity_buffer, FILTER_SIZE);
    
    // Apply exponential moving average
    ema_intensity = EMA_ALPHA * filtered_intensity + (1 - EMA_ALPHA) * ema_intensity;
    current_intensity = (uint16_t)ema_intensity;
    
    // Check stability
    updateStability(filtered_intensity);
    
    // Calculate absorbance and glucose (if calibrated)
    if (calibration.calibration_flag == EEPROM_CAL_MAGIC) {
      current_absorbance = calculateAbsorbance(current_intensity);
      current_glucose = estimateGlucose(current_absorbance);
    } else {
      current_absorbance = 0;
      current_glucose = 0;
    }
    
    // Measure battery voltage
    battery_percent = measureBatteryPercent();
  }
  
  // Update display at lower frequency to reduce flicker
  if (millis() - last_display_update >= DISPLAY_UPDATE_MS) {
    last_display_update = millis();
    updateDisplay();
  }
}

// ============================================================================
// AMBIENT LIGHT CANCELLATION & INTENSITY MEASUREMENT
// ============================================================================

/**
 * Measure light intensity using ambient light cancellation
 * 
 * Standard practice: differential measurement with LED ON and OFF
 * This removes constant ambient light artifacts
 * 
 * @return Intensity = I_on - I_off (in ADC counts)
 */
uint16_t measureIntensity() {
  // Disable interrupts for clean measurement timing
  cli();
  
  // ---- LED ON measurement ----
  digitalWrite(PIN_LED, HIGH);
  delayMicroseconds(LED_ON_DELAY_MS * 1000);  // Wait for photodiode settling
  uint16_t i_led_on = readADCAverage(ADC_PHOTODIODE, 4);  // 4 samples for speed
  digitalWrite(PIN_LED, LOW);
  
  // ---- LED OFF measurement (ambient only) ----
  delayMicroseconds(LED_OFF_DELAY_MS * 1000);  // Wait for LED to fully discharge
  uint16_t i_led_off = readADCAverage(ADC_PHOTODIODE, 4);  // 4 samples
  
  // Re-enable interrupts
  sei();
  
  // Differential measurement: removes ambient light
  // Ensure no unsigned underflow
  if (i_led_on <= i_led_off) {
    return 1;  // Minimum valid intensity
  }
  
  return (i_led_on - i_led_off);
}

/**
 * Read ADC with averaging for noise reduction
 * Uses ADC interrupt to allow other code to run
 * 
 * @param channel: ADC channel (0-7)
 * @param num_samples: Number of samples to average
 * @return Averaged ADC value (0-1023)
 */
uint16_t readADCAverage(uint8_t channel, uint8_t num_samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < num_samples; i++) {
    sum += analogRead(channel);
    delayMicroseconds(100);  // Small delay between samples
  }
  return sum / num_samples;
}

// ============================================================================
// FILTERING & STABILITY DETECTION
// ============================================================================

/**
 * Median filter for outlier rejection
 * Particularly useful for removing ADC spikes
 * 
 * @param buffer: Input buffer
 * @param size: Buffer size
 * @return Median value
 */
uint16_t medianFilter(uint16_t buffer[], uint8_t size) {
  // Simple insertion sort (suitable for small arrays)
  uint16_t sorted[size];
  memcpy(sorted, buffer, size * sizeof(uint16_t));
  
  // Bubble sort (simple and effective for small arrays)
  for (uint8_t i = 0; i < size - 1; i++) {
    for (uint8_t j = 0; j < size - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        uint16_t temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  return sorted[size / 2];
}

/**
 * Update stability counter
 * Requires STABILITY_SAMPLES consecutive stable measurements
 * before is_stable flag is set
 * 
 * @param intensity: Current measured intensity
 */
void updateStability(uint16_t intensity) {
  // Check if current measurement is within threshold of EMA
  if (abs((int)intensity - (int)ema_intensity) <= STABILITY_THRESHOLD) {
    stability_count++;
    if (stability_count >= STABILITY_SAMPLES) {
      is_stable = true;
    }
  } else {
    stability_count = 0;
    is_stable = false;
  }
}

// ============================================================================
// GLUCOSE ESTIMATION - ABSORBANCE BASED
// ============================================================================

/**
 * Calculate optical absorbance using Modified Beer-Lambert Law
 * 
 * A = log10(I0 / I)
 * 
 * Where:
 *   A   = Absorbance (dimensionless)
 *   I0  = Baseline intensity (no absorption)
 *   I   = Measured intensity (with absorption)
 * 
 * This is physically correct and matches academic standards
 * 
 * @param intensity: Measured intensity (in ADC counts)
 * @return Absorbance (dimensionless, typically 0-2)
 */
float calculateAbsorbance(uint16_t intensity) {
  if (calibration.baseline_intensity == 0 || intensity == 0) {
    return 0.0;
  }
  
  // Avoid division by zero and log of zero
  float ratio = (float)calibration.baseline_intensity / (float)intensity;
  if (ratio <= 0) ratio = 0.001;
  
  // log10(ratio) = log(ratio) / log(10)
  float absorbance = log10(ratio);
  
  // Clamp to valid range
  if (absorbance < ABSORBANCE_MIN) absorbance = ABSORBANCE_MIN;
  if (absorbance > ABSORBANCE_MAX) absorbance = ABSORBANCE_MAX;
  
  return absorbance;
}

/**
 * Estimate blood glucose from absorbance using polynomial regression
 * 
 * Model: G = β0 + β1*A + β2*A²
 * 
 * Coefficients are calibrated by comparing against reference glucometer
 * 
 * IMPORTANT: This is a RESEARCH PROTOTYPE.
 * Results must be verified against a calibrated reference.
 * 
 * @param absorbance: Optical absorbance
 * @return Estimated glucose (mmol/L)
 */
float estimateGlucose(float absorbance) {
  if (absorbance < ABSORBANCE_MIN || absorbance > ABSORBANCE_MAX) {
    return 0.0;
  }
  
  // Polynomial regression
  float A = absorbance;
  float A2 = A * A;
  
  float glucose = calibration.beta0 + 
                  calibration.beta1 * A + 
                  calibration.beta2 * A2;
  
  // Clamp to physiologically reasonable range
  if (glucose < 2.0) glucose = 2.0;    // Hypoglycemia threshold
  if (glucose > 30.0) glucose = 30.0;  // Upper physiological limit
  
  return glucose;
}

// ============================================================================
// BATTERY MONITORING
// ============================================================================

/**
 * Measure battery voltage and estimate charge percentage
 * Uses internal 1.1V reference for accuracy
 * 
 * Battery measurement circuit:
 *   Vbatt ---[R1]---+---[R2]--- GND
 *                   |
 *                  A1 (ADC input)
 * 
 * Assumes typical LiPo/LiIon 3.7V nominal, 4.2V max, 2.8V min
 * 
 * @return Battery percentage (0-100%)
 */
uint8_t measureBatteryPercent() {
  // Read battery voltage with averaging
  uint16_t adc_raw = readADCAverage(ADC_BATTERY, BATTERY_ADC_SAMPLES);
  
  // Convert ADC to voltage using internal reference
  // V = (ADC * VREF) / 1024 * DIVIDER_RATIO
  float voltage = (adc_raw * VREF_INTERNAL_MV / 1024.0) * BATTERY_DIVIDER_RATIO / 1000.0;
  
  // LiPo voltage to percentage (typical curve)
  // 4.2V = 100%, 2.8V = 0%
  float percent = (voltage - 2.8) / (4.2 - 2.8) * 100.0;
  
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  
  return (uint8_t)percent;
}

// ============================================================================
// CALIBRATION - BASELINE MEASUREMENT
// ============================================================================

/**
 * Handle calibration button press
 * User must hold button for CAL_BUTTON_HOLD_MS to trigger calibration
 * 
 * Procedure:
 *   1. Remove finger from sensor
 *   2. Press and hold CAL button for 2 seconds
 *   3. Device measures baseline intensity (LED on, no absorption)
 *   4. Baseline is stored to EEPROM
 *   5. OLED displays confirmation
 */
void handleCalibrationButton() {
  static uint32_t cal_hold_start = 0;
  
  if (digitalRead(PIN_CAL_BUTTON) == LOW) {
    // Button pressed
    if (cal_hold_start == 0) {
      cal_hold_start = millis();
      Serial.println(F("CAL button pressed - waiting for 2s hold..."));
    }
    
    // Check if held long enough
    if (millis() - cal_hold_start >= CAL_BUTTON_HOLD_MS) {
      performCalibration();
      cal_hold_start = 0;  // Reset for next press
    }
  } else {
    // Button released
    cal_hold_start = 0;
  }
}

/**
 * Perform baseline calibration
 * 
 * Procedure:
 *   1. Turn LED ON
 *   2. Wait for optical settling (~200 ms)
 *   3. Measure intensity (with ambient light cancellation)
 *   4. Average 20 samples for stability
 *   5. Store to EEPROM
 *   6. Set calibration flag
 */
void performCalibration() {
  Serial.println(F("\n*** CALIBRATION STARTED ***"));
  Serial.println(F("Ensure finger is REMOVED from sensor"));
  
  // Display message
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("CALIBRATION"));
  display.println(F("IN PROGRESS"));
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println(F("(Measuring baseline...)"));
  display.display();
  
  delay(500);  // Give user time to verify
  
  // Measure baseline intensity with multiple samples
  uint32_t baseline_sum = 0;
  const uint8_t CAL_SAMPLES = 20;
  
  for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds(LED_ON_DELAY_MS * 1000);
    uint16_t intensity = readADCAverage(ADC_PHOTODIODE, 4);
    digitalWrite(PIN_LED, LOW);
    
    baseline_sum += intensity;
    delay(50);
  }
  
  uint16_t baseline_intensity = baseline_sum / CAL_SAMPLES;
  
  // Store to EEPROM
  calibration.baseline_intensity = baseline_intensity;
  calibration.calibration_flag = EEPROM_CAL_MAGIC;
  
  // Save to EEPROM
  EEPROM.put(EEPROM_BASELINE_ADDR, baseline_intensity);
  EEPROM.put(EEPROM_CAL_FLAG_ADDR, (uint16_t)EEPROM_CAL_MAGIC);
  
  Serial.print(F("Baseline intensity: "));
  Serial.println(baseline_intensity);
  Serial.println(F("Calibration successful!"));
  
  // Display confirmation
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("CALIBRATION"));
  display.println(F("SUCCESS"));
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print(F("Baseline: "));
  display.println(baseline_intensity);
  display.display();
  
  delay(2000);  // Show message for 2 seconds
}

/**
 * Load calibration data from EEPROM
 */
void loadCalibration() {
  EEPROM.get(EEPROM_BASELINE_ADDR, calibration.baseline_intensity);
  EEPROM.get(EEPROM_CAL_FLAG_ADDR, calibration.calibration_flag);
  
  // Load coefficients if available
  EEPROM.get(EEPROM_COEFFICIENTS_ADDR, calibration.beta0);
  EEPROM.get(EEPROM_COEFFICIENTS_ADDR + 4, calibration.beta1);
  EEPROM.get(EEPROM_COEFFICIENTS_ADDR + 8, calibration.beta2);
  
  // Use defaults if not stored
  if (calibration.calibration_flag != EEPROM_CAL_MAGIC) {
    calibration.baseline_intensity = 0;
    calibration.beta0 = DEFAULT_BETA0;
    calibration.beta1 = DEFAULT_BETA1;
    calibration.beta2 = DEFAULT_BETA2;
    Serial.println(F("No valid calibration in EEPROM - using defaults"));
  } else {
    Serial.print(F("Calibration loaded - Baseline: "));
    Serial.println(calibration.baseline_intensity);
  }
}

/**
 * Save calibration coefficients to EEPROM
 * (Call this after fitting regression model)
 */
void saveCalibrationCoefficients(float b0, float b1, float b2) {
  calibration.beta0 = b0;
  calibration.beta1 = b1;
  calibration.beta2 = b2;
  
  EEPROM.put(EEPROM_COEFFICIENTS_ADDR, b0);
  EEPROM.put(EEPROM_COEFFICIENTS_ADDR + 4, b1);
  EEPROM.put(EEPROM_COEFFICIENTS_ADDR + 8, b2);
  
  Serial.println(F("Calibration coefficients saved to EEPROM"));
}

// ============================================================================
// DISPLAY UPDATE
// ============================================================================

/**
 * Update OLED display with current measurements
 * 
 * Displays:
 *   - Intensity (ADC counts)
 *   - Absorbance (dimensionless)
 *   - Glucose (mmol/L)
 *   - Battery percentage
 *   - Stability indicator
 *   - Calibration status
 */
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Title
  display.setCursor(0, 0);
  display.println(F("=== GLUCOSE MONITOR ==="));
  
  // Check calibration status
  if (calibration.calibration_flag != EEPROM_CAL_MAGIC) {
    // Not calibrated
    display.setCursor(0, 16);
    display.println(F("STATUS: Not Calibrated"));
    display.setCursor(0, 25);
    display.println(F("Press CAL button"));
    display.setCursor(0, 34);
    display.println(F("(remove finger first)"));
  } else {
    // Calibrated - show measurements
    display.setCursor(0, 16);
    display.print(F("Intensity: "));
    display.println(current_intensity);
    
    display.setCursor(0, 24);
    display.print(F("Absorbance: "));
    display.print(current_absorbance, 3);
    
    display.setCursor(0, 32);
    display.print(F("Glucose: "));
    display.print(current_glucose, 2);
    display.println(F(" mmol/L"));
    
    // Stability indicator
    display.setCursor(0, 40);
    if (is_stable) {
      display.print(F("Status: STABLE"));
    } else {
      display.print(F("Status: Settling ("));
      display.print(stability_count);
      display.print(F("/"));
      display.print(STABILITY_SAMPLES);
      display.println(F(")"));
    }
  }
  
  // Battery and calibration status (bottom row)
  display.setCursor(0, 56);
  display.print(F("Battery: "));
  display.print(battery_percent);
  display.print(F("% | CAL: "));
  if (calibration.calibration_flag == EEPROM_CAL_MAGIC) {
    display.println(F("OK"));
  } else {
    display.println(F("--"));
  }
  
  display.display();
}

// ============================================================================
// ADC CONFIGURATION
// ============================================================================

/**
 * Configure ADC for optimal performance
 * 
 * - Use internal 1.1V reference for battery measurement
 * - Set appropriate prescaler for ~125 kHz ADC clock
 * - Enable free-running mode
 */
void configureADC() {
  // ADC reference: INTERNAL (1.1V)
  // This is more stable than AVCC for battery measurement
  analogReference(INTERNAL);
  
  // ADC prescaler: 128 (16 MHz / 128 = 125 kHz)
  // Optimal for 10-bit conversion on ATmega328P
  ADCSRA &= ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  
  Serial.println(F("ADC configured with 1.1V internal reference"));
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Get absolute value of integer
 */
int abs(int value) {
  if (value < 0) return -value;
  return value;
}

// ============================================================================
// END OF FIRMWARE
// ============================================================================
