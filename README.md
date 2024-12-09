This is the home of the Smart Dipstick project!

This project works on the Adafruit Feather M0 BLE microcontroller.

This project takes samples of oil temperature and a voltage of oil in order to determine if oil is good, in need of changing soon, or if it is bad and in need of dire change. It sends Bluetooth Low Energy signals to work with our Android application.

With the Version 1.1 Update, our project now runs with a FreeRTOS library to collect samples and send BLE signals simultaneously for better readings and signal outputs to the app.

Version 1.2 update removes the bad oil database as we now use a percentage increase based upon the good oil value database to determine if the oil is good or not. Works more consistently and will allow for better general inclusion for future oil integrations.

Future updates to this project would include:
- Code related to an SD card for storing a wider database of oil values
- Updated code to allow users to initialize their own oil from the app
