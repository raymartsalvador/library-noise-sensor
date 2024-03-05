#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns and 2 rows

const int sampleWindow = 50;   // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

#define SENSOR_PIN A0
#define BUZZER_PIN 2  // Change this to the pin where your buzzer is connected
#define UP_BUTTON_PIN 3  // Change this to the pin where your UP button is connected
#define DOWN_BUTTON_PIN 5  // Change this to the pin where your DOWN button is connected

int sensitivity = 50;  // Initial sensitivity value, adjust as needed
bool adjustingSensitivity = false;  // Flag to indicate if sensitivity is being adjusted
unsigned long lastButtonPress = 0; 
void setup()
{
  pinMode(SENSOR_PIN, INPUT);   // Set the signal pin as input
  pinMode(BUZZER_PIN, OUTPUT);  // Set the buzzer pin as output
  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);  // Set UP button pin as input with internal pull-up resistor
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);  // Set DOWN button pin as input with internal pull-up resistor

  lcd.init(); // initialize the lcd
  lcd.backlight();

  Serial.begin(9600);
}

void loop()
{  unsigned long startMillis = millis();  // Start of the sample window
  float peakToPeak = 0;                 // Peak-to-peak level

  unsigned int signalMax = 0;           // Minimum value
  unsigned int signalMin = 1024;        // Maximum value

  // Collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(SENSOR_PIN);  // Get reading from the microphone
    if (sample < 1024)                 // Toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // Save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // Save just the min levels
      }
    }
  }

  peakToPeak = signalMax - signalMin;                 // Max - min = peak-peak amplitude
  int db = map(peakToPeak, 20, 900, 30, 120);         // Calibrate for deciBels with a wider range
  int adjustedDb = db + sensitivity;  // Adjusted loudness with sensitivity

  Serial.print("Loudness: ");
  Serial.print(adjustedDb);
  Serial.println("dB");

  // Display message on LCD based on sensitivity adjustment status
  lcd.clear();
  lcd.setCursor(0, 0);
  
  if (adjustingSensitivity) {
    lcd.print("Adjusting Sens.");
    lcd.setCursor(0, 1);
    lcd.print("Sensitivity: ");
    lcd.print(sensitivity);
  } else {
    lcd.print("Library Noise");
    lcd.setCursor(0, 1);
    lcd.print("DETECTOR");
  }

  if (adjustedDb >= 80)  // Adjust this threshold as needed
  {
    digitalWrite(BUZZER_PIN, HIGH);  // Activate the buzzer
    delay(1000);  // Buzzer on for 1 second (adjust as needed)
    digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer
  }

  // Calibration process
  if (digitalRead(UP_BUTTON_PIN) == HIGH && millis() - lastButtonPress > 200) {
    // If UP button is pressed and enough time has passed since the last button press
    lastButtonPress = millis();  // Update the last button press time
    adjustingSensitivity = true;
    sensitivity++;
    if (sensitivity > 50)
      sensitivity = 50;  // Ensure sensitivity does not exceed 50
  }

  if (digitalRead(DOWN_BUTTON_PIN) == HIGH && millis() - lastButtonPress > 200) {
    // If DOWN button is pressed and enough time has passed since the last button press
    adjustingSensitivity = true;
    lastButtonPress = millis();  // Update the last button press time
    sensitivity--;
    if (sensitivity < 0)
      sensitivity = 0;  // Ensure sensitivity does not go below 0
  }


  // Reset the flag when no button is pressed
  if (digitalRead(UP_BUTTON_PIN) == HIGH && digitalRead(DOWN_BUTTON_PIN) == HIGH) {
    adjustingSensitivity = false;
  }

  delay(50);  // Adjust this delay for smoother button response
}