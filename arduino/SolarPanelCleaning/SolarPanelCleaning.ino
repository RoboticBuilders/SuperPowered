#include <Stepper.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include "RTClib.h"
#include "LowPower.h"

// Standard Pins in Arduino
// *I2C Protocol*:
// SDA - data line - A4
// SCL - clock - A5
// These can be added to multiple perpherals. Used here for LightSensor and RTC
// ------------------------
// *SPI* protocol
// MOSI - digital 11
// MISO - digital 12
// SCK - digital 13
// CS - can be any pin, 10 used in this code
// This protocol is being used for reading/writing SD card
// -------------------------

// **Curuit Creation and Pins used**
// Light Sensor - GND, VIN (3.3V), A4 (SDA), A5 (SCL) (NOTE: A4 and A5 are shared with RTC module)
// RTC Module - GND, VIN(3.3V), A4 (SDA), A5 (SCL) (NOTE: this is exactly same as LightSensor)
// SD Module - GND, VCC(*5V* not 3.3 V), MISO (d12), MOSI (d11), SCK (d13), CS 
// Other Pins arrangements for LCD and Stepper Motor are written next to their declaration below
RTC_DS3231 rtc;

// pin for voltage sensor
#define ANALOG_IN_VOLTAGE_PIN A0

//sd card variables
const unsigned long writeFrequencyinSeconds = 600;
DateTime lastDataSaveTime = new DateTime((int)0);

volatile int sleep_count = 0; // Keep track of how many sleep cycles have been completed.
const int sleep_total = (writeFrequencyinSeconds)/8; // Approximate number of sleep cycles needed before the interval defined above elapses. Note that this does integer math.
int CS_PIN = 10;

// uncommentline below to test with I2C LCD, just setting it to zero or 1  doesn't work
// #define USE_LCD_I2C 0

// adc voltage
float adc_voltage = 0.0;

// resistor values in the sensor
const float R1 = 30000.0;
const float R2 = 7500.0;

// reference voltage - can be less if arduino is doing more things at that moment
const float ref_voltage = 5.0;

// integer for adc value
int adc_value = 0;

#ifdef USE_LCD_I2C
LCD_I2C lcd(0x27, 16, 2);
#else


// Potentiometer
// Bottom left on Potentiometer comes from Positive Rail (5v)
// Bottom right on Potentionmeter goes to Negative Rail (Ground)
// Top Potentiometer goes to V0

// VSS goes to negative rail
// VDD goes to positive rail
// V0 comes from potentiometer, as above
// RS as below
// RW in negative rail
// E as below for en
// Leave four pins between E and d4 unconnected
// d4 to d7 as below
// A positive rail
// K negative rail

// LCD Pins for non Solar Rollar, which is using SD card needed on pins 10, 11, 12, 13 (check above)
// These pins overlap with motors, but motor is not needed for Solar Rollar.
//const int rs = 7;
//const int en = 8;
//const int d4 = 5;
//const int d5 = 4;
//const int d6 = 3;
//const int d7 = 2;

// LCD Pins for Solar Rollar (Wooden as well as Metal) - same as in Paul McWhorter's video
const int rs = 7;
const int en = 8;
const int d4 = 9;
const int d5 = 10;
const int d6 = 11;
const int d7 = 12;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#endif

// char part numbers
const int LeftTopTrimmedChar = 1;
const int RightTopTrimmedChar = 2;
const int LeftBottomTrimmedChar = 3;
const int RightBottomTrimmedChar = 4;
const int TopBarChar = 5;
const int BottomBarChar = 6;
const int UpperAndMiddleBarPartChar = 7;
const int DecimalChar = 8;
const int CompleteCellChar = 255;

// Stepper motor
const int STEPS_PER_REV = 240;
const float voltage_threshold = 2.0;
const float lux_threshold = 20;
const int numberOfRotationsForOneLength = 5;
const int motorSpeed = 20;

// Time to cool off before considering another change
// 10 minutes
const unsigned long coolOffTimeInMilliseconds = 600000; 

unsigned long lastTimeSheetWasChanged = 0;

// Motor Pins
const int motorIn1 = 2;
const int motorIn2 = 3;
const int motorIn3 = 4;
const int motorIn4 = 5;
Stepper stepper(STEPS_PER_REV, motorIn1, motorIn2, motorIn3, motorIn4);

// Light sensor TSL2591
// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// Conditional elements in the circuit
const bool circuitHasLCD = true;
const bool circuitHasLightSensor = false;
const bool circuitHasRTC = false;
const bool circuitHasSDCard = false;
const bool circuitSetForDataCollection = false;
const bool circuitHasMotor = true;

void setup() {
  Serial.begin(9600);
  if (circuitHasRTC == true) {
    initializeRTC();
  }


 if (circuitHasLightSensor) { 
  initializeLightSensor();
 }
    
  if (circuitHasSDCard) {
      initializeSDCard();
  }

  if (circuitHasLCD) {
 #ifdef USE_LCD_I2C   
   lcd.begin();
   lcd.backlight();
  #else
    lcd.begin(16,2);
  #endif
    initializeDigitParts();
  }
}

void loop() {
    if (circuitSetForDataCollection && (sleep_count < sleep_total))
    {
      //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
      //    SPI_OFF, USART0_OFF, TWI_OFF);
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      sleep_count++;
      return;
    } 

    sleep_count = 0;    
    delay(100);  
    
    DateTime now =  new DateTime((int)0);
    
    float temperature = 0.0;

    if (circuitHasRTC) { 
      now = rtc.now(); 
      temperature = rtc.getTemperature();
    }

    long luxValue = 0;
    if (circuitHasLightSensor) {
      luxValue = (long)getLuminosity();
    }

    float in_voltage = readAndDisplayVoltage();
    
    //Serial.println(in_voltage);

    if (circuitSetForDataCollection) {
      writeToFile("arduino.txt", now, luxValue, in_voltage, temperature);
    }
      
    char data = Serial.read();
    bool onDemandChange = false;

    if (data == '1')
    {
      onDemandChange = true;
    }
    else if (data == '2')
    {
      Serial.println(in_voltage);
    }

    if (circuitHasMotor && (onDemandChange || (in_voltage < voltage_threshold && circuitHasLightSensor && luxValue > lux_threshhold && isTimeToTriggerChange())))
    {
      changeProtectionSheet();
    }
  
    delay(1000);
}


void initializeLightSensor()
{
  if (tsl.begin())
  {
    Serial.println(F("Found a TSL2591 sensor"));
  }
  else
  {
    Serial.println(F("No sensor found ... check your wiring?"));
    while (1); // stop and don't proceed
  }
  
  tsl.setGain(TSL2591_GAIN_LOW); // 1x
  tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  Serial.println(F("Light sensor initialized!"));
}

//returns luminosity in lux
float getLuminosity()
{  
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  float luxValue = tsl.calculateLux(full, ir);
  return luxValue;
}

// determines if it is time to write data
bool isTimeToWriteData(DateTime now)
{
  if(lastDataSaveTime.year() == 1970 or (now.secondstime() - lastDataSaveTime.secondstime() > writeFrequencyinSeconds))
  {
    lastDataSaveTime = now;
    return true;
  }

  return false;
}

//SD card fuctions are over here
void initializeSDCard()
{
  Serial.println(F("Initializing SD card..."));
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  if (!SD.begin(CS_PIN)) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1); // don't do anything more:
  }

  Serial.println(F("SD CARD INITIALIZED."));
}


File openFile(char filename[])
{
  return SD.open(filename, FILE_WRITE);
}

// csv file line with following data in each line
// dateAndTime, luxValue, voltage, temperature
String formLineToWrite(DateTime dateTime, float luxValue, float voltage, float temperature)
{
      char bufDate[50];
      
      //keeping US date format      
      sprintf(bufDate, "%02d-%02d-%02d %02d:%02d:%02d", dateTime.year(), dateTime.month(), dateTime.day(), dateTime.hour(), dateTime.minute(), dateTime.second()); 

      String dateString = String(bufDate);
      String lineString = dateString + "," + luxValue + "," + voltage + "," + temperature;

      return lineString;
}

int writeToFile(char fileName[], DateTime dateTime, float luxValue, float voltage, float temperature)
{
  String logData = formLineToWrite(dateTime, luxValue, voltage, temperature);
  String logString = "Input string: " + logData; 
  //Serial.println(logString.c_str());
  
  File file = openFile(fileName);
 
  if (file)
  {
    char logDataText[logData.length()+1];
    logData.toCharArray(logDataText, logData.length()+1);

    // Serial.println("Writing to file: ");
    // Serial.println(logDataText);
  
    file.println(logDataText);

    closeFile(file);
    return 1;
  } else
  {
    Serial.println(F("Couldn't write to file"));
    return 0;
  }
}

void closeFile(File file)
{
  if (file)
  {
    file.close();
    //Serial.println("File closed");
  }
}


void initializeRTC()
{
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

bool isTimeToTriggerChange()
{
  
  unsigned long currentTime = millis();
  if (lastTimeSheetWasChanged == 0 ||  (currentTime - lastTimeSheetWasChanged) > coolOffTimeInMilliseconds)
  {
    lastTimeSheetWasChanged = currentTime;
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
      readAndDisplayVoltage();
      stepper.step(STEPS_PER_REV);
    }
}

float readAndDisplayVoltage()
{
  float in_voltage = readInputVoltage();

  if (circuitHasLCD) {
      String voltage(in_voltage, 2);
      displayVoltageOnLCD(voltage);
  }

  return in_voltage;
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

// This assumes we will only show single decimal place for a number using
// the custom big font
void displayVoltageOnLCD(String str)
{
  lcd.clear();

  // 4 chars assuming one for decimal
  // TBD: using array of functions will help
  int pos = 0;
  for (int i = 0; i < 4; i++)
  {
    char ch = str[i];
    switch (ch)
    {
      case '0':
        drawBigZero(pos);
        break;
      case '1':
        drawBigOne(pos);
        break;
      case '2':
        drawBigTwo(pos);
        break;
      case '3':
        drawBigThree(pos);
        break;
      case '4':
        drawBigFour(pos);
        break;
      case '5':
        drawBigFive(pos);
        break;
      case '6':
        drawBigSix(pos);
        break;
      case '7':
        drawBigSeven(pos);
        break;
      case '8':
        drawBigEight(pos);
        break;
      case '9':
        drawBigNine(pos);
        break;
      case '.':
        drawBigDecimal(pos);
        break;
      default:
        // error
        // don't do a thing
        break;
    }

    pos += 4;
  }
}

  // Bigger font chars to be drawn for voltage
  // Each character is 3 cell columns by two cell rows (with each cell being 40 pixels (8rows * 5 columns))
  // segments of chars
  byte LeftTopTrimmed[8] = 
  {
    B00111,
    B01111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111
  };

  byte RightTopTrimmed[8] =
  {
    B11100,
    B11110,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111
  };

  byte LeftBottomTrimmed[8] =
  {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B01111,
    B00111
  };

  byte RightBottomTrimmed[8] =
  {
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11110,
    B11100
  };

  byte TopBar[8] =
  {
    B11111,
    B11111,
    B11111,
    B00000,
    B00000,
    B00000,
    B00000,
    B00000
  };

  byte BottomBar[8] =
  {
    B00000,
    B00000,
    B00000,
    B00000,
    B00000,
    B11111,
    B11111,
    B11111
  };

  byte UpperAndMiddleBarPart[8] =
  {
    B11111,
    B11111,
    B11111,
    B00000,
    B00000,
    B00000,
    B11111,
    B11111
  };

  byte Decimal[8] =
  {
    B00000,
    B00000,
    B01110,
    B11111,
    B11111,
    B11111,
    B01110,
    B00000
  };

// code to draw digits
void initializeDigitParts()
{
  lcd.createChar(LeftTopTrimmedChar, LeftTopTrimmed);
  lcd.createChar(RightTopTrimmedChar, RightTopTrimmed);
  lcd.createChar(LeftBottomTrimmedChar, LeftBottomTrimmed);
  lcd.createChar(RightBottomTrimmedChar, RightBottomTrimmed);
  lcd.createChar(TopBarChar, TopBar);
  lcd.createChar(BottomBarChar, BottomBar);
  lcd.createChar(UpperAndMiddleBarPartChar, UpperAndMiddleBarPart);
  lcd.createChar(DecimalChar, Decimal);
}

void drawBigZero(int position)
{
  lcd.setCursor(position,0);
  lcd.write(LeftTopTrimmedChar);
  lcd.write(TopBarChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position, 1);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(BottomBarChar);
  lcd.write(RightBottomTrimmedChar);  
}

void drawBigOne(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(TopBarChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position, 1);
  lcd.write(BottomBarChar);
  lcd.write(CompleteCellChar);
  lcd.write(BottomBarChar);    
}

void drawBigTwo(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position, 1);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(BottomBarChar);
  lcd.write(BottomBarChar);
}

void drawBigThree(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position, 1);
  lcd.write(BottomBarChar);
  lcd.write(BottomBarChar);
  lcd.write(RightBottomTrimmedChar);
}

void drawBigFour(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(BottomBarChar);
  lcd.write(CompleteCellChar);
  lcd.setCursor(position + 2, 1);
  lcd.write(CompleteCellChar);
}

void drawBigFive(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.setCursor(position , 1);
  lcd.write(BottomBarChar);
  lcd.write(BottomBarChar);
  lcd.write(RightBottomTrimmedChar);  
}

void drawBigSix(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(LeftTopTrimmedChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.setCursor(position , 1);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(BottomBarChar);
  lcd.write(RightBottomTrimmedChar);
}

void drawBigSeven(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(TopBarChar);
  lcd.write(TopBarChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position + 2 , 1);
  lcd.write(CompleteCellChar);
}

void drawBigEight(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(LeftTopTrimmedChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position, 1);
  lcd.write(LeftBottomTrimmedChar);
  lcd.write(BottomBarChar);
  lcd.write(RightBottomTrimmedChar);
}

void drawBigNine(int position)
{
  lcd.setCursor(position, 0);
  lcd.write(LeftTopTrimmedChar);
  lcd.write(UpperAndMiddleBarPartChar);
  lcd.write(RightTopTrimmedChar);
  lcd.setCursor(position + 2, 1);
  lcd.write(CompleteCellChar);
}

void drawBigDecimal(int position)
{
  lcd.setCursor(position + 1, 1);
  lcd.write(DecimalChar);
}
