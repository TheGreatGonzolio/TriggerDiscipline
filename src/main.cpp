
/*****************************************************************************/
/*INCLUDES                                                                   */
/*****************************************************************************/
#include <Arduino.h>
#include <ArduinoBLE.h>

/* For all of the sensors */
#include "Nano33BLEAccelerometer.h"
#include "Nano33BLEGyroscope.h"
#include "Nano33BLEMagnetic.h"

/*****************************************************************************/
/*MACROS                                                                     */
/*****************************************************************************/

#define BLE_BUFFER_SIZES 20 // 20 bytes is max ble packet size (1 char = 1 byte)
#define BLE_DEVICE_NAME "Arduino Nano 33 BLE Sense"
#define BLE_LOCAL_NAME "Sensors"

/*****************************************************************************/
/*GLOBAL Data                                                                */
/*****************************************************************************/

/*Sensor objects that we will instanciate in the setup */
Nano33BLEMagnetic mag;
Nano33BLEGyroscope gyro;
Nano33BLEAccelerometer accel;

/* Objects which we will store data in each time we read the each sensor.*/
Nano33BLEMagneticData magneticData;
Nano33BLEGyroscopeData gyroscopeData;
Nano33BLEAccelerometerData accelerometerData;

/* 
 * Declares the BLEService and characteristics we will need for the BLE 
 * transfer. The UUID was randomly generated using one of the many online 
 * tools that exist. It was chosen to use BLECharacteristic instead of 
 * BLEFloatCharacteristic (and other characteristic types) as it is hard 
 * to view non-string data in most BLE scanning software. Strings can be 
 * viewed easiler enough. In an actual application you might want to 
 * transfer specific data types directly.
 */
BLEService BLESensors("590d65c7-3a0a-4023-a05a-6aaf2f22441c");
BLECharacteristic magneticBLEX("0001", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic magneticBLEY("0002", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic magneticBLEZ("0003", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);

BLECharacteristic gyroscopeBLEX("0004", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic gyroscopeBLEY("0005", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic gyroscopeBLEZ("0006", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);

BLECharacteristic accelerometerBLEX("0007", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic accelerometerBLEY("0008", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);
BLECharacteristic accelerometerBLEZ("0009", BLERead | BLENotify | BLEBroadcast, BLE_BUFFER_SIZES);

/* Common global buffer will be used to write to the BLE characteristics. */
char bleBuffer[BLE_BUFFER_SIZES];




void setup()
{

  Serial.begin(115200);
  while (!Serial)
  {
    // Wait for serial connection.
  }

  if(!BLE.begin())
  {
    Serial.println("BLE Failed to Start! Device will not continue.");

    //TODO: Determine if there is a better way to handle bluetooth critical failure?
    while(true); // leave software at this point
  }

    BLE.setDeviceName(BLE_DEVICE_NAME);
    BLE.setLocalName(BLE_LOCAL_NAME);
    BLE.setAdvertisedService(BLESensors);

    /* A seperate characteristic is used for each sensor data type. */
    BLESensors.addCharacteristic(magneticBLEX);
    BLESensors.addCharacteristic(magneticBLEY);
    BLESensors.addCharacteristic(magneticBLEZ);
    BLESensors.addCharacteristic(gyroscopeBLEX);
    BLESensors.addCharacteristic(gyroscopeBLEY);
    BLESensors.addCharacteristic(gyroscopeBLEZ);
    BLESensors.addCharacteristic(accelerometerBLEX);
    BLESensors.addCharacteristic(accelerometerBLEY);
    BLESensors.addCharacteristic(accelerometerBLEZ);

    BLE.addService(BLESensors);
    BLE.advertise();

    Serial.printf("BLE service advertising!\nDevice Name: %s\n Local Name: %s\nService UUID: %s\n", BLE_DEVICE_NAME, BLE_LOCAL_NAME, BLESensors.uuid());

    //TODO: Verifiy that each sensor is on different thread.. Library seems to be not true
    /* Initialises the all the sensor, and starts the periodic reading of the sensor using a Mbed OS thread*/
    mag.begin();
    gyro.begin();
    accel.begin();

    /* Plots the legend on Serial Plotter */
    //Serial.println("MagX, MagY, MagZ, GyX, GyY, GyZ, AccX, AccY, AccZ");
    Serial.println("MagX, MagY, MagZ");

}

/*****************************************************************************/
/*LOOP (runtime super loop)                                                  */
/*****************************************************************************/
void loop()
{
  BLEDevice central = BLE.central();
  if (central)
  {
    int writeLength;
    bool dataGotFlag = false;

    while (central.connected())
    {
 
      if (mag.pop(magneticData))
      {
        writeLength = sprintf(bleBuffer, "%f", magneticData.x);
        magneticBLEX.writeValue(reinterpret_cast<byte const *> (bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", magneticData.y);
        magneticBLEY.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", magneticData.z);
        magneticBLEZ.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        dataGotFlag = true;
      }

      if (gyro.pop(gyroscopeData))
      {
        writeLength = sprintf(bleBuffer, "%f", gyroscopeData.x);
        gyroscopeBLEX.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", gyroscopeData.y);
        gyroscopeBLEY.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", gyroscopeData.z);
        gyroscopeBLEZ.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        dataGotFlag = true;
      }

      if (accel.pop(accelerometerData))
      {
        writeLength = sprintf(bleBuffer, "%f", accelerometerData.x);
        accelerometerBLEX.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", accelerometerData.y);
        accelerometerBLEY.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        writeLength = sprintf(bleBuffer, "%f", accelerometerData.z);
        accelerometerBLEZ.writeValue(reinterpret_cast<byte const *>(bleBuffer), writeLength);
        dataGotFlag = true;
      }

      if (dataGotFlag)
      {
        char serialOutput[90];
        // sprintf(serialOutput,
        //         "%f,%f,%f,%f,%f,%f,%f,%f,%f",
        //         magneticData.x,
        //         magneticData.y,
        //         magneticData.z,
        //         gyroscopeData.x,
        //         gyroscopeData.y,
        //         gyroscopeData.z,
        //         accelerometerData.x,
        //         accelerometerData.y,
        //         accelerometerData.z);

        sprintf(serialOutput,
                "%f,%f,%f",
                magneticData.x,
                magneticData.y,
                magneticData.z);

        //Serial.println(serialOutput);
        //Serial.printf("%f,%f,%f", magneticData.x, magneticData.x, magneticData.y, magneticData.z);
      }
    }
  }
}