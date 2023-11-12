// Arduino pins Connections----------------------------------------------------------------------------

// A0 - ACS712 to monitor solar current
// A1 - Voltage divider to measure solar panel
// A2 - Voltage divider to measure battery voltage
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

#define SOLAR_VOLT_SENS 1        // defining the analog pin A0 to read solar panel Voltage
#define BATTERY_VOLT_SENS 2      // defining the analog pin A1 to read battery voltage
#define SOLAR_CURRENT_SENS 0     // defining the Analog pin A2 to measure load current
#define PWM_PIN 3                // defining digital pin D3 to drive the main MOSFET

VoltageSensor Input_Voltage_Sensor(VOLTSENS_100K, SOLAR_VOLT_SENS);
VoltageSensor Battery_Voltage_Sensor(VOLTSENS_100K, BATTERY_VOLT_SENS);
ACS712 Current_Sensor(ACS712_30A, SOLAR_CURRENT_SENS);
LiquidCrystal_I2C LCD(0x27, 16, 2);

double Current_Value;
double Watts;

bool To_Charge;

void SerialPrint();
void LCD_Print();

void ReadValues();
void Charge();

void setup()
{
    Serial.begin(9600);
    Serial.println("BOOTING...");

    LCD.init();
    LCD.backlight();

    LCD.setCursor(0, 0);
    LCD.println("BOOTING...");

    LCD.createChar(1, solar);
    LCD.createChar(2, battery);
    LCD.createChar(3, energy);
    LCD.createChar(4, charge);
    LCD.createChar(5, not_charge);

    pinMode(SOLAR_VOLT_SENS, INPUT);
    pinMode(BATTERY_VOLT_SENS, INPUT);
    pinMode(SOLAR_CURRENT_SENS, INPUT);
    pinMode(PWM_PIN, OUTPUT);

    Current_Sensor.calibrate();

    delay(500);

    Serial.println("OK!");
    Serial.println();

    LCD.clear();

    LCD.setCursor(0, 0);
    LCD.println("OK!");

    delay(1000);
}

void loop()
{
    ReadValues();
    Charge();
    SerialPrint();
    LCD_Print();

    delay(1000);
}

void SerialPrint()
{
    Serial.println();
    Serial.print("Solar voltage - ");
    Serial.print(Input_Voltage_Sensor.GetValue());
    Serial.println();
    Serial.print("Battery voltage - ");
    Serial.print(Battery_Voltage_Sensor.GetValue());
    Serial.println();
    Serial.print("Current - ");
    Serial.print(Current_Value);
    Serial.println();
    Serial.print("Watts - ");
    Serial.print(Watts);
    Serial.println();
    Serial.print("Charge state - ");
    Serial.print(Input_Voltage_Sensor.GetValue());
    Serial.println();
}

void LCD_Print()
{
    LCD.clear();
    // Display Solar Panel Parameters
    LCD.setCursor(0, 0);
    LCD.write(1);
    LCD.setCursor(2, 0);
    LCD.print(Input_Voltage_Sensor.GetValue(), 1);
    LCD.print("V");
    LCD.setCursor(8, 0);
    LCD.print(Current_Value, 1);
    LCD.print("A");

    // Display Battery Parameters
    LCD.setCursor(0, 1);
    LCD.write(2);
    LCD.setCursor(2, 1);
    LCD.print(Battery_Voltage_Sensor.GetValue(), 1);
    LCD.print("V");
    LCD.setCursor(9, 1);
    LCD.print(Watts, 1);
    LCD.print("W");
    LCD.setCursor(15, 1);
    if(To_Charge)
    {
        LCD.write(4);
    }
    else
    {
        LCD.write(5);
    }
}

void ReadValues()
{
    if(To_Charge)
    {
        analogWrite(PWM_PIN, 0); //

        Input_Voltage_Sensor.ReadValue();

        delay(100); //
        analogWrite(PWM_PIN, 255); //
    }

    Battery_Voltage_Sensor.ReadValue();

    Current_Value = Current_Sensor.getCurrentDC();

    if(Current_Value < 0.16)
    {
        Current_Value = 0;
    }

    if(Input_Voltage_Sensor.GetValue() < 0.16)
    {
        Input_Voltage_Sensor.SetValue(0);
    }

    if(Battery_Voltage_Sensor.GetValue() < 0.16)
    {
        Battery_Voltage_Sensor.SetValue(0);
    }

    Watts = Current_Value * Battery_Voltage_Sensor.GetValue();

    if (Battery_Voltage_Sensor.GetValue() >= 14.4)
    {
        To_Charge = false;
    }
    else if (Battery_Voltage_Sensor.GetValue() <= 13.2)
    {
        To_Charge = true;
    }
}

void Charge()
{
    if(To_Charge)
    {
        analogWrite(PWM_PIN, 255);
    }
    else
    {
        analogWrite(PWM_PIN, 0);
    }
}