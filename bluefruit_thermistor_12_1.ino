/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


/**************************************************************************/

// Setup for the Thermistor

// Code for reading thermistor modified from:
// https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/

// analog pin A0 for reading thermistor
int ThermistorPin = 0;

// analog pin A1 for reading oil
int oilResitivityPin = 2;

// Voltage integer
int Vo;
float V1;

// Known Resistor 30K ohms
float R1 = 30000;

// Float values
// R2 = value of the Thermistor
// T = temperature measured from Thermistor
float R2, T;

// Constant values from datasheet for Thermistor
// Located here: https://www.vishay.com/en/thermistors/ntc-rt-calculator/
// Thermistor part #: NTCLE350E4303fhb0
float A = 0.00334382306701192;
float B = 0.000261842759267469;
float C = 3.92235933627021e-06;
float D = 1.39503244666293e-07;

// Circular buffers for storing the last 1000 samples
float tempSamples[100];
float oilSamples[1000];

// Ints for tracking indexes of circular buffers
volatile int tempIndex = 0;
volatile int oilIndex = 0;

// Boolean for when we can start sending bluetooth data
bool bufferFilled = false;

// Declare floats for total averages of temp and oil voltage
volatile float totalTemp = 0.0;
volatile float totalOil = 0.0;

// helps delay time between sending bluetooth data to phone app
volatile int sendDelay = 0;

// Databases of Oil temps and oil voltages for both good and bad oil.
// Our databases will be in degrees celsius and be for 5 degree measures

// For anything between our good maximum and bad minimum voltage measures,
// we find that the oil is close to needing a change

// Struct for how good oil data gets stored
// Good oil only needs a max voltage threshold before we consider it no longer good
struct oilGoodDatabase {
  float tempMin;
  float tempMax;
  float voltMax;
};

// Struct for how bad oil data gets stored
// Bad oil only needs a minimum voltage threshold before we consider it to be bad
struct oilBadDatabase {
  float tempMin;
  float tempMax;
  float voltMin;
};

// Based upon the 17 oil ranges we took values of
const int range = 17;

oilGoodDatabase goodOil[range] = {
  { 20.0, 25.0, -0.15 },
  { 25.0, 30.0, -0.2 },
  { 30.0, 35.0, -0.29 },
  { 35.0, 40.0, -0.38 },
  { 40.0, 45.0, -0.47 },
  { 45.0, 50.0, -0.6 },
  { 50.0, 55.0, -0.71 },
  { 55.0, 60.0, -0.87 },
  { 60.0, 65.0, -1.01 },
  { 65.0, 70.0, -1.19 },
  { 70.0, 75.0, -1.39 },
  { 75.0, 80.0, -1.62 },
  { 80.0, 85.0, -1.86 },
  { 85.0, 90.0, -2.13 },
  { 90.0, 95.0, -2.51 },
  { 95.0, 100.0, -3.0 },
  { 100.0, 105.0, -3.21 }
};

// Should be same temp ranges as good oil, but different voltage ranges
oilBadDatabase badOil[range] = {
  { 20.0, 25.0, -0.16 },
  { 25.0, 30.0, -0.21 },
  { 30.0, 35.0, -0.3 },
  { 35.0, 40.0, -0.4 },
  { 40.0, 45.0, -0.5 },
  { 45.0, 50.0, -0.6 },
  { 50.0, 55.0, -0.8 },
  { 55.0, 60.0, -1.0 },
  { 60.0, 65.0, -1.2 },
  { 65.0, 70.0, -1.4 },
  { 70.0, 75.0, -1.7 },
  { 75.0, 80.0, -2.0 },
  { 80.0, 85.0, -2.4 },
  { 85.0, 90.0, -2.8 },
  { 90.0, 95.0, -3.1 },
  { 95.0, 100.0, -3.25 },
  { 100.0, 105.0, -3.25 } // Cutoff at 3.25 since becoming too close to 3.3V
};


// Function for determining if Oil is good, bad, or close to needing change


/**************************************************************************/

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // Setup perfomred over next 3 lines so that microcontroller sends out
  // values without need of connection to computer serial terminal
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000); // Wait up to 5 seconds

  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  // Begin setup for the circular buffers:

  // Initialize tempSamples with 100 elements
  for (int i = 0; i < 100; i++) {
      tempSamples[i] = 0.0;
  }

  // Initialize oilSamples with 1000 elements
  for (int i = 0; i < 1000; i++) {
      oilSamples[i] = 0.0;
  }

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

// Helper function to determine if oil condition is good, in need of change soon, or in need of change now
int oilCheck(float avgTempC, float avgOilVoltage)
{
  // Start by finding which set of temperature ranges the current oil temp falls into
  for (int i = 0; i < range; i++) 
  {
    if (avgTempC >= goodOil[i].tempMin && avgTempC <= goodOil[i].tempMax) 
    {
      // Case where there is actually not enough oil to measure
      if (avgOilVoltage >= 0.00)
      {
        return 3; // Oil is unmeasureable as the dipstick is not submersed in oil
      }
      // If greater than the max, oil is good
      if (avgOilVoltage >= goodOil[i].voltMax) 
      {
        return 0;  // Oil is good, no change needed
      }
      // If lower than the min, oil is bad
      if (avgOilVoltage <= badOil[i].voltMin) 
      {
        return 2;  // Oil is bad, needs a change
      }
    }
  }
  // Otherwise, oil is inbetween max good and min bad, so we are close to needing change
  return 1;  // Default to "close to needing a change" if no specific range matches
}

// Main loop
void loop(void)
{
  // Timing measurements so that data is only send every 2 seconds total
  static unsigned long timeNow = 0;
  static unsigned long timeTotal = 0;

  // Read the voltage from the analog pins
  Vo = analogRead(ThermistorPin);
  V1 = analogRead(oilResitivityPin);

  // Obtain resistance of thermistor
  R2 = R1 * (1023.0 / (float)Vo - 1.0);

  // Calucate the temperature in Celsius (From Thermistor Manufacturer)
  T = 1 / (A + B * log(R2 / R1) + C * pow(log(R2 / R1), 2) + D * pow(log(R2 / R1), 3)) - 273.15;

  // Store the new sample in the buffer
  tempSamples[tempIndex] = T;
  oilSamples[oilIndex] = (((V1 * 3300.0F/1024.0F / 1000)-3.3)*(10000+20000))/10000+3.3;

  // Reset indexes properly as we go through our loop over time
  tempIndex = (tempIndex + 1) % 100;
  oilIndex = (oilIndex + 1) % 1000;

  // Check if the oil buffer is fully filled since it is the longest to fill
  if (oilIndex == 0) {
    bufferFilled = true;
  }

  // Calculate the average if the buffer is filled
  // Send delay will make sure 0.5 seconds pass before trying to send data for the bluetooth again
  if (bufferFilled && (sendDelay >= 50))
  {
    totalTemp = 0.0;  // Immediately reset after average calculation
    totalOil = 0.0;

    // cycle through to obtain the 100 current values in circular buffer and then add them all together
    for (int i = 0; i < 100; i++)
    {
      totalTemp += tempSamples[i];
    }

    // cycle through to obtain the 1000 current values in circular buffer and then add them all together
    for (int i = 0; i < 1000; i++)
    {
      totalOil += oilSamples[i];
    }

    // Find average temperature from current 100 samples
    float avgTempC = totalTemp / 100;

    // Find average oil voltage from current 1000 samples
    float avgOilVoltage = totalOil/1000.0;

    // Print the average temperature
    // Print the average temperature in Celsius
    Serial.print("Oil Temp: ");
    Serial.print(avgTempC, 2);
    Serial.println(" degrees C");
    // Convert from C to F and print it
    float avgTempF = (avgTempC * 9.0) / 5.0 + 32.0;
    // Print the average temperature in Fahrenheit
    Serial.print("Oil Temp: ");
    Serial.print(avgTempF, 2);
    Serial.println(" degrees F");
    Serial.print("Voltage of the oil: ");
    Serial.println(avgOilVoltage, 4);

    //Send input data to host via Bluefruit

    // Status is an int (essentially enum) that is sent to the app
    // which outputs a message based on what state it recieves:
    // 0 = oil is good, no change needed
    // 1 = oil is bad, change needed soon
    // 2 = oil is bad, needs a change!
    // 3 = need more oil!

    //INT good WILL BE BASED UPON THE READING OF TEMPERATURE AND VOLTAGE INPUT FROM THE CICRUIT FROM THE OIL
    int good = oilCheck(avgTempC, avgOilVoltage);

    // Output information to the bluetooth device
    ble.print(String(good) + "#");
    ble.print(String(avgTempF) + "#");
    ble.print(String(avgTempC) + "#");
    //ble.print(avgOilVoltage, 4);
    ble.println("cfrm");

    // Reset the total temp
    totalTemp = 0.0;
    totalOil = 0.0;

    //Reset the Send Delay
    sendDelay = -1;
  }

  // Wait 10 ms before taking the next sample to maintain 1000 samples per second
  delay(10);
  sendDelay++;
}