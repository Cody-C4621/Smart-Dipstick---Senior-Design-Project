// Smart Dipstick V1.1 Code
// Created by the Smart Dipstick Team at Georgia Tech Fall 2024
// Cody Ells, Daniel Luna, Ori Krasnovsky, Michael McDonald, Leo Lin

// Uses FreeRtos library for SAMD21 created by Scott Briscoe
// and nRF51822 library created by Adafruit

// Runs on the Adafruit Feather M0 Bluefruit LE Microcontroller

//**************************************************************************
// FreeRtos on Samd21
// By Scott Briscoe
//**************************************************************************

#include <FreeRTOS_SAMD21.h>

//**************************************************************************
// Type Defines and Constants
//**************************************************************************

// Select the serial port the project should use and communicate over
// Some boards use SerialUSB, some use Serial
#define MY_SERIAL          Serial //Adafruit, other Samd21 Boards (What we desire for Adafruit's Feather M0 boards)

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

//**************************************************************************
// global variables
//**************************************************************************
TaskHandle_t Handle_sampleCollection;
TaskHandle_t Handle_bluetoothOutput;

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

// Databases of Oil temps and oil voltages for both good oil.
// Our databases will be in degrees celsius and be for 5 degree measures

// Struct for how good oil data gets stored
// Good oil only needs a max voltage threshold before we consider it no longer good
struct oilGoodDatabase {
  float tempMin;
  float tempMax;
  float voltMax;
};

// Based upon the 34 oil ranges we took values of
const int range = 34;

oilGoodDatabase goodOil[range] = {
    { 22.5, 25.0, -0.1 },
    { 25.0, 27.5, -0.2 },
    { 27.5, 30.0, -0.25 },
    { 30.0, 32.5, -0.28 },
    { 32.5, 35.0, -0.32 },
    { 35.0, 37.5, -0.36 },
    { 37.5, 40.0, -0.4 },
    { 40.0, 42.5, -0.445 },
    { 42.5, 45.0, -0.49 },
    { 45.0, 47.5, -0.54 },
    { 47.5, 50.0, -0.59 },
    { 50.0, 52.5, -0.645 },
    { 52.5, 55.0, -0.7 },
    { 55.0, 57.5, -0.74 },
    { 57.5, 60.0, -0.83 },
    { 60.0, 62.5, -0.925 },
    { 62.5, 65.0, -0.975 },
    { 65.0, 67.5, -1.06 },
    { 67.5, 70.0, -1.135 },
    { 70.0, 72.5, -1.2 },
    { 72.5, 75.0, -1.46 },
    { 75.0, 77.5, -1.57 },
    { 77.5, 80.0, -1.65 },
    { 80.0, 82.5, -1.73 },
    { 82.5, 85.0, -1.9 },
    { 85.0, 87.5, -2.05 },
    { 87.5, 90.0, -2.2 },
    { 90.0, 92.5, -2.4 },
    { 92.5, 95.0, -2.6 },
    { 95.0, 97.5, -2.75 },
    { 97.5, 100.0, -2.9 },
    { 100.0, 102.5, -3.0 },
    { 102.5, 105.0, -3.1 }
};

//**************************************************************************
// Can use these function for RTOS delays
// Takes into account processor speed
// Use these instead of delay(...) in rtos tasks											
//**************************************************************************

void myDelayMs(int ms)
{
  vTaskDelay( (ms * 1000) / portTICK_PERIOD_US );  
}

//**************************************************************************

// Thread for taking oil samples
static void sampleCollection( void *pvParameters ) 
{
  while(true)
  {
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

    // Wait 10 ms before taking next sample
    myDelayMs(10);
  }
}

// Thread for sending data over BLE
static void bluetoothOutput( void *pvParameters ) 
{
  while(true)
  {
    // Calculate the average if the buffer is filled
    // Send delay will make sure 0.5 seconds pass before trying to send data for the bluetooth again
    if (bufferFilled)
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

    Serial.println(F("******************************"));
    Serial.println(F("Outputting new Readings!"));
    Serial.println(F("******************************"));

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

    Serial.println(F("******************************"));
    Serial.println(F("******************************"));

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

    // Wait 500 before sending new info to phone
    myDelayMs(500);
  }
}

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

  delay(1000); // prevents usb driver crash on startup, do not omit this

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

  // Create the threads that will be managed by the rtos
  // Sets the stack size and priority of each task
  // Also initializes a handler pointer to each task, which are important to communicate with and retrieve info from tasks
  xTaskCreate(sampleCollection, "sampleCollection", 1024, NULL, tskIDLE_PRIORITY + 1, &Handle_sampleCollection);
  xTaskCreate(bluetoothOutput, "bluetoothOutput", 1024, NULL, tskIDLE_PRIORITY, &Handle_bluetoothOutput);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();

  // error scheduler failed to start
  // should never get here
  while(1)
  {
	  Serial.println("Scheduler Failed! \n");
	  Serial.flush();
	  delay(1000);
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
      if (avgOilVoltage <= goodOil[i].voltMax * 1.18) 
      {
        return 2;  // Oil is bad, needs a change
      }
    }
  }
  // Otherwise, oil is inbetween max good and min bad, so we are close to needing change
  return 0;  // Default to "oil is good" if no specific range matches
}

// Main loop (Empty as all functionality is run through threads)
void loop(void)
{
}




