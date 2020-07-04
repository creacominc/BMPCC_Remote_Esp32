#include "BMPCCAdvertisedDeviceCallbacks.h"
#include <HardwareSerial.h>
#include <BLEDevice.h>

BMPCCAdvertisedDeviceCallbacks::BMPCCAdvertisedDeviceCallbacks( BLEAdvertisedDevice ** _Camera,
                                                                bool * _deviceFound,
                                                                BLEUUID & _cameraControlServiceUUID )
  : BLEAdvertisedDeviceCallbacks()
  , m_Camera( _Camera )
  , m_deviceFound( _deviceFound )
  , m_cameraControlServiceUUID( _cameraControlServiceUUID )
{
}

BMPCCAdvertisedDeviceCallbacks::~BMPCCAdvertisedDeviceCallbacks()
{
}


void BMPCCAdvertisedDeviceCallbacks::onResult(BLEAdvertisedDevice advertisedDevice)
{
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str());
  // We have found a device, let us now see if it contains the service we are looking for.
  if (advertisedDevice.haveServiceUUID()
      && advertisedDevice.isAdvertisingService( m_cameraControlServiceUUID ))
  {
    // stop scanning
    BLEDevice::getScan()->stop();
    Serial.println("found camera.");
    (*m_Camera) = new BLEAdvertisedDevice( advertisedDevice );
    (*m_deviceFound) = true;
  }
}
