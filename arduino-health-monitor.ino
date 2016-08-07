/*********************************************************************
This code was based off of the starting examples from Adafruit. 

Andrew McKittrick | 2016 
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"

#include "BluefruitConfig.h"

#include "IEEE11073float.h"

// Create the bluefruit object, either software serial...uncomment these lines

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEGatt gatt(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */

int32_t htsServiceId;
int32_t htsMeasureCharId;
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;

  Serial.begin(115200);
  Serial.println(F("Health Monitoring System | Andrew McKittrick | 2016"));
  Serial.println(F("--------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'AEM Health Monitor': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=AEM Health Monitor")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Thermometer Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heath Thermometer Service definition (UUID = 0x1809): "));
  htsServiceId = gatt.addService(0x1809);
  if (htsServiceId == 0) {
    error(F("Could not add Thermometer service"));
  }

  /* Add the Temperature Measurement characteristic which is composed of
   * 1 byte flags + 4 float */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Temperature Measurement characteristic (UUID = 0x2A1C): "));
  htsMeasureCharId = gatt.addCharacteristic(0x2A1C, GATT_CHARS_PROPERTIES_INDICATE, 5, 5, BLE_DATATYPE_BYTEARRAY);
  if (htsMeasureCharId == 0) {
    error(F("Could not add Temperature characteristic"));
  }

  /* Add the Thermometer Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Thermometer Service UUID to the advertising payload: "));
  uint8_t advdata[] { 0x02, 0x01, 0x06, 0x05, 0x02, 0x09, 0x18, 0x0a, 0x18 };
  ble.setAdvData( advdata, sizeof(advdata) );

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    error(F("Could not add HRM service"));
  }
  
  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
    if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
    if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );


  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();
}

/** Send randomized heart rate data continuously **/
void loop(void)
{
  double temp = random(0, 100) / 10.0;
  int heart_rate = random(50, 100);

  Serial.print(F("Updating Temperature value to "));
  Serial.print(temp);
  Serial.println(F(" Fahrenheit"));

  // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature_measurement.xml
  // Chars value is 1 flag + 4 float value. Tempearature is in Fahrenheit unit
  uint8_t temp_measurement [5] = { bit(0) };
  
  float2IEEE11073(temp, temp_measurement+1);

  // TODO temperature is not correct due to Bluetooth use IEEE-11073 format
  gatt.setChar(htsMeasureCharId, temp_measurement, 5);

  Serial.print(F("Updating HRM value to "));
  Serial.print(heart_rate);
  Serial.println(F(" BPM"));

  /* Command is sent when \n (\r) or println is called */
  /* AT+GATTCHAR=CharacteristicID,value */
  ble.print( F("AT+GATTCHAR=") );
  ble.print( hrmMeasureCharId );
  ble.print( F(",00-") );
  ble.println(heart_rate, HEX);

  /* Check if command executed OK */
  if ( !ble.waitForOK() )
  {
    Serial.println(F("Failed to get response!"));
  }

  /* Delay before next measurement update */
  delay(1000);
}
