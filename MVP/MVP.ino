#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_VL53L0X.h"
#include "AS5600_PsW.h"

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

// Pin Definitions
#define BUTTON_PIN 6
#define CYCLE_BUTTON_PIN 4
#define SHARP_SENSOR_PIN A0

// Sharp IR Sensor Calibration
#define SHARP_SAMPLES 5             // Number of samples for averaging
#define SHARP_VOLTAGE_REF 5.0       // Reference voltage
#define SHARP_ADC_RESOLUTION 1023.0 // ADC resolution

// Kalman Filter Parameters
#define PROCESS_NOISE 0.01
#define MEASUREMENT_NOISE 0.1

// Kalman Filter Structure
struct KalmanFilter
{
  float estimate;
  float error_estimate;
  float process_noise;
  float measurement_noise;
};

// Initialize Kalman Filters
KalmanFilter kalmanSmall = {0, 1, PROCESS_NOISE, MEASUREMENT_NOISE};
KalmanFilter kalmanLong = {0, 1, PROCESS_NOISE, MEASUREMENT_NOISE};

// Initialize Components
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
AS5600_PsW Sensor;

// Global Variables
float zeroReference = 0.0;
bool buttonPressed = false;
int mode = 1; // 1: Small Distance, 2: Long Distance, 3: String Distance

// Turn Tracking Variables
int prevRaw = 0;
float relativeAngle = 0;
bool firstRead = true;

// Kalman Filter Update Function
float kalmanUpdate(KalmanFilter *kf, float measurement)
{
  // Prediction
  kf->error_estimate += kf->process_noise;

  // Update
  float kalman_gain = kf->error_estimate / (kf->error_estimate + kf->measurement_noise);
  kf->estimate += kalman_gain * (measurement - kf->estimate);
  kf->error_estimate *= (1 - kalman_gain);

  return kf->estimate;
}

// Function to get averaged Sharp IR sensor reading
float getSharpDistance()
{
  float sum = 0;
  for (int i = 0; i < SHARP_SAMPLES; i++)
  {
    sum += analogRead(SHARP_SENSOR_PIN);
    delay(10); // Small delay between readings
  }
  float average = sum / SHARP_SAMPLES;
  float voltage = average * (SHARP_VOLTAGE_REF / SHARP_ADC_RESOLUTION);

  // More accurate formula for Sharp GP2Y0A21YK0F
  // Distance in cm = 29.988 * pow(voltage, -1.173)
  float distance_cm = 29.988 * pow(voltage, -1.173);

  // Apply calibration offset if needed
  distance_cm -= 0.5; // Small calibration offset

  return kalmanUpdate(&kalmanLong, distance_cm);
}

void setup()
{
  Serial.begin(115200);

  // Initialize buttons
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(CYCLE_BUTTON_PIN, INPUT_PULLUP);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println(F("SSD1306 failed"));
    while (1)
      ;
  }

  // Initialize VL53L0X
  if (!lox.begin())
  {
    Serial.println(F("VL53L0X failed"));
    while (1)
      ;
  }

  // Initialize AS5600
  Sensor.init();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void loop()
{
  // Check mode cycling button
  if (digitalRead(CYCLE_BUTTON_PIN) == LOW)
  {
    mode = (mode % 3) + 1; // Cycle through 1, 2, 3
    delay(200);            // Debounce
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (mode == 1)
  {
    // Small Distance Mode (VL53L0X)
    display.print("Small Distance");

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4)
    {
      float mm = measure.RangeMilliMeter - 10.0; // Calibration offset
      mm = kalmanUpdate(&kalmanSmall, mm);

      // Zero reference button handling
      if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed)
      {
        zeroReference = mm;
        buttonPressed = true;
      }
      else if (digitalRead(BUTTON_PIN) == HIGH)
      {
        buttonPressed = false;
      }

      float relative_mm = mm - zeroReference;
      float relative_cm = relative_mm / 10.0;
      float inches = relative_mm / 25.4;
      float feet = inches / 12.0;

      // Display distance measurements
      display.setTextSize(2);
      display.setCursor(0, 14);
      display.print("CM: ");
      display.print(relative_cm, 2);

      display.setTextSize(1);
      display.setCursor(0, 36);
      display.print("MM: ");
      display.print(relative_mm, 2);

      display.setCursor(0, 48);
      display.print("FT: ");
      display.print(feet, 2);
      display.print(" | IN: ");
      display.print(inches, 2);
      Serial.println(relative_cm);
    }
    else
    {
      display.setCursor(0, 20);
      display.print("Out of range");
    }
  }
  else if (mode == 2)
  {
    // Long Distance Mode (Sharp IR)
    display.print("Long Distance");

    float distance_cm = getSharpDistance();

    // Zero reference button handling
    if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed)
    {
      zeroReference = distance_cm;
      buttonPressed = true;
    }
    else if (digitalRead(BUTTON_PIN) == HIGH)
    {
      buttonPressed = false;
    }

    float relative_cm = distance_cm - zeroReference;
    float relative_mm = relative_cm * 10;
    float inches = relative_cm / 2.54;
    float feet = inches / 12.0;

    // Display distance measurements
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print("CM: ");
    display.print(relative_cm, 2);

    display.setTextSize(1);
    display.setCursor(0, 36);
    display.print("MM: ");
    display.print(relative_mm, 2);

    display.setCursor(0, 48);
    display.print("FT: ");
    display.print(feet, 2);
    display.print(" | IN: ");
    display.print(inches, 2);
  }
  else
  {
    // String Distance Mode (AS5600)
    display.print("String Distance");

    int raw = Sensor.rawAngle();

    if (firstRead)
    {
      prevRaw = raw;
      firstRead = false;
    }
    else
    {
      int diff = raw - prevRaw;

      // Handle wrap-around
      if (diff > 2048)
        diff -= 4096;
      if (diff < -2048)
        diff += 4096;

      // Update relative angle
      float diffDeg = (diff * 360.0) / 4096.0;
      relativeAngle += diffDeg;

      prevRaw = raw;

      // Calculate string length
      float totalTurns = relativeAngle / 360.0;
      float stringLength_cm = totalTurns * PI * 3.0; // Length = turns * π * 3
      float stringLength_mm = stringLength_cm * 10.0;

      // Display measurements
      display.setTextSize(2);
      display.setCursor(0, 14);
      display.print("CM: ");
      display.print(stringLength_cm, 2);

      display.setTextSize(1);
      display.setCursor(0, 36);
      display.print("MM: ");
      display.print(stringLength_mm, 2);

      // Zero reference button handling
      if (digitalRead(BUTTON_PIN) == LOW && !buttonPressed)
      {
        relativeAngle = 0;
        buttonPressed = true;
      }
      else if (digitalRead(BUTTON_PIN) == HIGH)
      {
        buttonPressed = false;
      }
    }
  }

  display.display();
  delay(50);
}
