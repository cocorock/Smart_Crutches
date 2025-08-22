// Modified ESP32 Code - Push Buttons and Analog Reading
// Compatible with Arduino IDE Serial Plotter

//---- Global Variables and Definitions ----
String file_name = "Program: ESP32_ButtonsAnalog.ino";

//---- Push Buttons Configuration ----
const int keyPadPins[] = { 4, 0, 2, 15 };  // Push button pins
const int numButtons = sizeof(keyPadPins) / sizeof(int);
const int ledPin = 5;  // LED pin

//---- Analog Reading Configuration ----
const int analogPin = 33;  // Analog input pin
int analogValue = 0;       // Variable to store analog reading

//---- State Machine Definitions ----
enum State {
  STATE_1,   // Default state - Button 1
  STATE_2,   // Button 2 pressed
  STATE_3,   // Button 3 pressed  
  STATE_4    // Button 4 pressed
};

State currentState = STATE_1;  // Initialize state
State lastButtonPressed = STATE_1;  // Track last button pressed

//---- Timing Variables ----
unsigned long lastAnalogRead = 0;
const unsigned long analogReadInterval = 100;  // Read analog every 100ms
unsigned long lastLedToggle = 0;
const unsigned long ledToggleInterval = 500;   // Toggle LED every 500ms

//---- Setup Function ----
void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
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
  
  Serial.println("System initialized");
  Serial.println("Button1\tAnalogValue");  // Headers for Arduino Plotter
  
  delay(1000);  // Give time for serial monitor to connect
}

//---- Main Loop Function ----
void loop() {
  // Handle button state machine
  handleButtons();
  
  // Read analog value periodically
  if (millis() - lastAnalogRead >= analogReadInterval) {
    analogValue = analogRead(analogPin);
    lastAnalogRead = millis();
    
    // Print data compatible with Arduino Plotter
    // Format: ButtonValue\tAnalogValue
    printPlotterData();
  }
  
  // Toggle LED periodically for visual feedback
  if (millis() - lastLedToggle >= ledToggleInterval) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    lastLedToggle = millis();
  }
  
  delay(10);  // Small delay to prevent watchdog resets
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
      break;
      
    case STATE_2:
      // Button 2 pressed
      Serial.println("Button 2 pressed");
      break;
      
    case STATE_3:
      // Button 3 pressed
      Serial.println("Button 3 pressed");
      break;
      
    case STATE_4:
      // Button 4 pressed
      Serial.println("Button 4 pressed");
      break;
  }
}

//---- Print Data for Arduino Plotter ----
void printPlotterData() {
  // Convert button state to numeric value for plotter
  int buttonValue = static_cast<int>(lastButtonPressed) + 1;  // 1-4 instead of 0-3
  
  // Print in Arduino Plotter format: Value1\tValue2
  Serial.print(buttonValue);
  Serial.print("\t");
  Serial.println(analogValue);
}

//---- Alternative Print Function (Detailed) ----
void printDetailedData() {
  // Uncomment this function call in loop() if you want detailed output instead
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print("\tLast Button: ");
  Serial.print(static_cast<int>(lastButtonPressed) + 1);
  Serial.print("\tAnalog Pin 33: ");
  Serial.println(analogValue);
}
