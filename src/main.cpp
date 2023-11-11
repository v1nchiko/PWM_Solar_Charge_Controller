// Arduino pins Connections----------------------------------------------------------------------------

// A0 - Voltage divider to measure solar panel voltage
// A1 - Voltage divider to measure battery voltage
// A2 - ACS712 to monitor solar current
// A3 - Not used
// A4 - LCD SDA
// A5 - LCD SCL

// D0 - Not used
// D1 - Not used
// D2 - Not used
// D3 - PWM Output to control MOSFET
// D4 - Not Used
// D5 - Not used
// D6 - Not used
// D7 - Not used
// D8 - Not used
// D9 - Not used
// D10 - Not used
// D11 - Not used
// D12 - Not used
// D13 - not used


// Include Libraries

#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "ACS712.h"
#include "Sensors/VoltageSensor.h"

//--------------------------------------------------------------------------------------------------------------------------
///////////////////////DECLARATION OF ALL BIT MAP ARRAY FOR FONTS///////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------
byte solar[8] = {0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000}; //icon for solar panel
byte battery[8] = {0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111}; //icon for battery
byte energy[8] = {0b00010,0b00100,0b01000,0b11111,0b00010,0b00100,0b01000,0b00000}; // icon for power
byte charge[8] = {  0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100}; // icon for battery charge
byte not_charge[8]= {0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000}; // icon for battery non charge

//---------------------------------------------------------------------------------------------------------------------------

// Definitions :

#define SOLAR_VOLT_SENS 0        // defining the analog pin A0 to read solar panel Voltage
#define BATTERY_VOLT_SENS 1      // defining the analog pin A1 to read battery voltage
#define SOLAR_CURRENT_SENS 2     // defining the Analog pin A2 to measure load current
#define PWM_PIN 3                // defining digital pin D3 to drive the main MOSFET

VoltageSensor InputVoltageSensor(VOLTSENS_100K, SOLAR_VOLT_SENS);
VoltageSensor BatteryVoltageSensor(VOLTSENS_100K, BATTERY_VOLT_SENS);
ACS712 CurrentSensor(ACS712_30A, SOLAR_CURRENT_SENS);
LiquidCrystal_I2C lcd(0x27, 16,2);

void SerialPrint();
void LCD_Print();

void setup()
{
    Serial.begin(9600);
    Serial.println("BOOTING...");

    lcd.init();
    lcd.backlight();

    lcd.createChar(1, solar);
    lcd.createChar(2, battery);
    lcd.createChar(3, energy);
    lcd.createChar(4, charge);
    lcd.createChar(5, not_charge);

    lcd.setCursor(0,0);
    lcd.println("BOOTING...");

    Serial.println("OK!");
    Serial.println();

    lcd.setCursor(0,0);
    lcd.println("OK!");

    delay(1000);
}

void loop()
{
// write your code here
}