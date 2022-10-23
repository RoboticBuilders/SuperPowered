#include <Stepper.h>
#include <LiquidCrystal.h>

#define ANALOG_IN_VOLTAGE_PIN A0

// adc voltage
float adc_voltage = 0.0;

//actual voltage we are measuring
float in_voltage = 0.0;

// resistor values in the sensor
const float R1 = 30000.0;
const float R2 = 7500.0;

// reference voltage - can be less if arduino is doing more things at that moment
const float ref_voltage = 5.0;

// integer for adc value
int adc_value = 0;


// LCD Pins
const int rs = 7;
const int en = 8;
const int d4 = 9;
const int d5 = 10;
const int d6 = 11;
const int d7 = 12;

// LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Stepper motor
const int STEPS_PER_REV = 240;
const float voltage_threshold = 2.0;
const int numberOfRotationsForOneLength = 5;
const int motorSpeed = 20;

// Time to cool off before considering another change
// 10 minutes
const unsigned long coolOffTime = 600000; 

unsigned long lastTimeSheetWasChanged = 0;

Stepper stepper(STEPS_PER_REV, 2, 3, 4, 5);


void setup() {
  Serial.begin(9600);
  Serial.println("Starting solar cleaning!");

  Serial.println("Starting LCD");
  lcd.begin(16,2);
}

void loop() {
    Serial.print("Input Voltage = ");
    in_voltage = readInputVoltage();
    Serial.println(in_voltage, 2);

    String voltage(in_voltage, 2);
    displayStringOnLCD(voltage);

    if (in_voltage < voltage_threshold && isTimeToTriggerChange())
    {
      changeProtectionSheet();
    }

    delay(1000);
}

bool isTimeToTriggerChange()
{
  unsigned long time = millis();
  Serial.print("Time = ");
  Serial.println(time);
  Serial.print("LastTimeSheetWasChanged = ");
  Serial.println(lastTimeSheetWasChanged);
  Serial.print("coolOffTime = ");
  Serial.println(coolOffTime);
  if (lastTimeSheetWasChanged == 0 ||  (time - lastTimeSheetWasChanged) > coolOffTime)
  {
    lastTimeSheetWasChanged = time;
    return true;
  }

  return false;
}
// this method rotates motor to change sheet by one length
// of solar panel and plus
void changeProtectionSheet()
{
    stepper.setSpeed(motorSpeed);
    for(int i = 0; i < numberOfRotationsForOneLength; i++)
    {
      stepper.step(STEPS_PER_REV);
    }
}

float readInputVoltage()
{
  adc_value = analogRead(ANALOG_IN_VOLTAGE_PIN);
  adc_voltage = (adc_value * ref_voltage) / 1024.0;
  return adc_voltage / (R2 / (R1+R2) );
}

void displayStringOnLCD(String str)
{
  lcd.setCursor(0, 0);
  lcd.print(str);
}
