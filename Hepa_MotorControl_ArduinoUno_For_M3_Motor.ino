#include <Wire.h>
#include <PCF8574.h>

// Define pins on Arduino Uno for motor speed control
// #define MEDIUM_SPEED_PIN 8  // Corresponds to P5 on PCF8574
#define PCF_PIN_7 7    // Corresponds to P6 on PCF8574
#define PCF_PIN_5 6    // Corresponds to P7 on PCF8574

// Define motor control parameters
#define TRIAC_PIN 5         // Triac control pin
#define MEDIUM_PHASE_DELAY 5000
#define HIGH_PHASE_DELAY 3000
#define LOW_PHASE_DELAY 6500

volatile unsigned long phase_delay = 0; // Default: Motor OFF
volatile bool zc_detected = false;      // Zero crossing flag

// Track previous states of pins
bool prevHighSpeed = HIGH;
bool prevLowSpeed = HIGH;

void setup() {
  // Initialize speed control pins as input
  pinMode(PCF_PIN_7, INPUT);
  pinMode(PCF_PIN_5, INPUT);
  // digitalWrite(MEDIUM_SPEED_PIN, HIGH); // Enable internal pull-up resistors
  // digitalWrite(PCF_PIN_7, HIGH);
  // digitalWrite(PCF_PIN_5, HIGH);

  // Initialize triac control pin
  pinMode(TRIAC_PIN, OUTPUT);
  digitalWrite(TRIAC_PIN, LOW);

  // Attach interrupt for Zero Crossing Detector (ZCD)
  attachInterrupt(digitalPinToInterrupt(2), zeroCrossInterrupt, RISING);
  setPhaseDelay(0); // Stop motor
  // Initialize Serial for debugging
  Serial.begin(9600);
}

int pin_5_read, pin_7_read;

void loop() {
  // Read the current states of speed pins
  pin_7_read = digitalRead(PCF_PIN_7);
  pin_5_read = digitalRead(PCF_PIN_5);

  Serial.print("Pin 7: ");
  Serial.println(pin_7_read);
  Serial.print("Pin 5: ");
  Serial.println(pin_5_read);
  Serial.println();
  // Check for high speed activation
  if (pin_5_read == 0 && pin_7_read == 0) {
    setPhaseDelay(HIGH_PHASE_DELAY); // 3_WiFi_Highest_Speed
    Serial.println("Highest Speed");
  }
  else if (pin_5_read == 1 && pin_7_read == 0) {
    setPhaseDelay(LOW_PHASE_DELAY); // 1_WiFiStartUp Low speed
    Serial.println("Low speed");
  }
  else if (pin_5_read == 0 && pin_7_read == 1) {
    setPhaseDelay(MEDIUM_PHASE_DELAY); //2_WiFi Medium speed
    Serial.println("Medium speed");
  }
  else if (pin_5_read == 1 && pin_7_read == 1) {
    setPhaseDelay(0); // Motor OFF
    Serial.println("Sleep Mode Motor OFF");
  }

  delay(500); // Small delay to avoid unnecessary processing
}

void zeroCrossInterrupt() {
  zc_detected = true;                // Set zero-crossing flag
  if (phase_delay > 0) {             // Trigger triac only if phase_delay > 0
    delayMicroseconds(phase_delay);  // Wait for phase delay
    digitalWrite(TRIAC_PIN, HIGH);   // Trigger triac
    delayMicroseconds(10);           // Maintain pulse width
    digitalWrite(TRIAC_PIN, LOW);    // Turn off triac gate
  }
}

void setPhaseDelay(unsigned long newDelay) {
  // Update the phase delay to the selected speed or stop motor if delay is 0
  phase_delay = newDelay;
}
