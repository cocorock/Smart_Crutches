// Modified ESP32 Code - Push Buttons, Analog Reading, and Touch Sensor
// Compatible with Arduino IDE Serial Plotter

//---- Global Variables and Definitions ----
String file_name = "Program: ESP32_ButtonsAnalogTouch.ino";

//---- Push Buttons Configuration ----
const int keyPadPins[] = { 4, 0, 15, 2 };  // Push button pins
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;  // LED pin

//---- Analog Reading Configuration ----
const int analogPin = 33;  // Analog input pin
int analogValue = 0;       // Variable to store analog reading

//---- Touch Sensor Configuration ----
const int touchPin = T5;   // GPIO12 (Touch5) - T5 is predefined constant
int touchThreshold = 30;   // Threshold for touch detection (adjust as needed)
int touchValue = 0;        // Variable to store touch reading
bool touchDetected = false; // Touch state
bool lastTouchState = false; // Previous touch state for edge detection

//---- State Machine Definitions ----
enum State {
  STATE_1,   // Default state - Button 1
  STATE_2,   // Button 2 pressed
  STATE_3,   // Button 3 pressed  
  STATE_4,   // Button 4 pressed
  STATE_TOUCH // Touch sensor activated
};

State currentState = STATE_1;  // Initialize state
State lastButtonPressed = STATE_1;  // Track last button pressed

//---- Timing Variables ----
unsigned long lastAnalogRead = 0;
const unsigned long analogReadInterval = 100;  // Read analog every 100ms
unsigned long lastLedToggle = 0;
const unsigned long ledToggleInterval = 500;   // Toggle LED every 500ms
unsigned long lastTouchRead = 0;
const unsigned long touchReadInterval = 50;    // Read touch every 50ms

//---- Setup Function ----
void setup() {
  // Initialize Serial Communication
  Serial.begin(921600);
  while (!Serial) {}
  Serial.println(file_name);
  
  // Initialize push button pins as INPUT_PULLUP
  for (int i = 0; i < numButtons; i++) {
    pinMode(keyPadPins[i], INPUT_PULLUP);
  }
  
  // Initialize LED pin as output
  pinMode(ledPin, OUTPUT);
  
  // Initialize analog pin (though it's input by default)
  pinMode(analogPin, INPUT);
  
  // No need to initialize touch pin - it's handled by ESP32 touch API
  
  Serial.println("System initialized");
  Serial.println("Touch sensor on GPIO12 (T5) configured");
  Serial.println("Touch threshold set to: " + String(touchThreshold));
  Serial.println("Button\tAnalogValue\tTouchValue");  // Headers for Arduino Plotter
  
  delay(1000);  // Give time for serial monitor to connect
}

//---- Main Loop Function ----
void loop() {
  // Handle touch sensor reading
  if (millis() - lastTouchRead >= touchReadInterval) {
    handleTouchSensor();
    lastTouchRead = millis();
  }
  
  // Handle button state machine
  handleButtons();
  
  // Read analog value periodically
  if (millis() - lastAnalogRead >= analogReadInterval) {
    analogValue = analogRead(analogPin);
    lastAnalogRead = millis();
    
    // Print data compatible with Arduino Plotter
    printPlotterData();
  }
  
  // Toggle LED periodically for visual feedback
  if (millis() - lastLedToggle >= ledToggleInterval) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    lastLedToggle = millis();
  }
  
  delay(10);  // Small delay to prevent watchdog resets
}

//---- Touch Sensor Handling Function ----
void handleTouchSensor() {
  touchValue = touchRead(touchPin);  // Read touch value
  
  // Check if touch is detected (value goes below threshold when touched)
  touchDetected = (touchValue < touchThreshold);
  
  // Detect touch press (rising edge)
  if (touchDetected && !lastTouchState) {
    currentState = STATE_TOUCH;
    handleStateActions();
    Serial.println("Touch detected! Value: " + String(touchValue));
  }
  
  // Detect touch release (falling edge)
  if (!touchDetected && lastTouchState) {
    Serial.println("Touch released! Value: " + String(touchValue));
  }
  
  lastTouchState = touchDetected;
}

//---- Button Handling Function ----
void handleButtons() {
  static unsigned long lastButtonTime = 0;
  const unsigned long debounceDelay = 50;  // 50ms debounce
  
  // Check if enough time has passed since last button press
  if (millis() - lastButtonTime < debounceDelay) {
    return;
  }
  
  // Check each button
  for (int i = 0; i < numButtons; i++) {
    if (digitalRead(keyPadPins[i]) == LOW) {  // Button pressed (active low)
      currentState = static_cast<State>(i);
      lastButtonPressed = currentState;
      lastButtonTime = millis();
      
      // Handle state actions
      handleStateActions();
      break;  // Exit loop after first button found
    }
  }
}

//---- State Actions Handler ----
void handleStateActions() {
  switch (currentState) {
    case STATE_1:
      // Button 1 pressed
      Serial.println("Button 1 pressed");
      lastButtonPressed = STATE_1;
      break;
      
    case STATE_2:
      // Button 2 pressed
      Serial.println("Button 2 pressed");
      lastButtonPressed = STATE_2;
      break;
      
    case STATE_3:
      // Button 3 pressed
      Serial.println("Button 3 pressed");
      lastButtonPressed = STATE_3;
      break;
      
    case STATE_4:
      // Button 4 pressed
      Serial.println("Button 4 pressed");
      lastButtonPressed = STATE_4;
      break;
      
    case STATE_TOUCH:
      // Touch sensor activated
      Serial.println("Touch sensor activated");
      // You can add specific touch actions here
      // For example: change LED pattern, trigger special function, etc.
      digitalWrite(ledPin, HIGH);  // Turn on LED when touched
      break;
  }
}

//---- Print Data for Arduino Plotter ----
void printPlotterData() {
  // Convert button/touch state to numeric value for plotter
  int stateValue;
  
  if (touchDetected) {
    stateValue = 5;  // Touch sensor value
  } else {
    stateValue = static_cast<int>(lastButtonPressed) + 1;  // 1-4 for buttons
  }
  
  // Print in Arduino Plotter format: Value1\tValue2\tValue3
  Serial.print(stateValue);
  Serial.print("\t");
  Serial.print(analogValue);
  Serial.print("\t");
  Serial.println(touchValue);
}

//---- Touch Calibration Function ----
void calibrateTouchSensor() {
  // Call this function if you need to calibrate the touch threshold
  Serial.println("Calibrating touch sensor...");
  Serial.println("Don't touch the sensor for 3 seconds...");
  
  delay(1000);
  
  long sum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    sum += touchRead(touchPin);
    delay(30);
  }
  
  int baselineValue = sum / samples;
  touchThreshold = baselineValue * 0.8;  // Set threshold to 80% of baseline
  
  Serial.println("Calibration complete!");
  Serial.println("Baseline value: " + String(baselineValue));
  Serial.println("New threshold: " + String(touchThreshold));
}

//---- Alternative Print Function (Detailed) ----
void printDetailedData() {
  // Uncomment this function call in loop() if you want detailed output instead
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print("\tLast Button: ");
  Serial.print(static_cast<int>(lastButtonPressed) + 1);
  Serial.print("\tAnalog Pin 33: ");
  Serial.print(analogValue);
  Serial.print("\tTouch Value: ");
  Serial.print(touchValue);
  Serial.print("\tTouch Detected: ");
  Serial.println(touchDetected ? "YES" : "NO");
}