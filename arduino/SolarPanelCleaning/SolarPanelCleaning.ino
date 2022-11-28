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
  initializeDigitParts();
  lcd.begin(16,2);
}

void loop() {
    Serial.print("Input Voltage = ");
    in_voltage = readInputVoltage();
    Serial.println(in_voltage, 2);

    String voltage(in_voltage, 2);
    displayVoltageOnLCD(voltage);

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

// This assumes we will only show single decimal place for a number using
// the custom big font
void displayVoltageOnLCD(String str)
{
  int pos = 0;
  lcd.clear();

  // 4 chars assuming one for decimal
  // TBD: using array of functions will help
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