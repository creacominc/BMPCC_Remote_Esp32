#include "BMPCCAdvertisedDeviceCallbacks.h"

#include <functional>

#include <BLEAddress.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>


BMPCCAdvertisedDeviceCallbacks::BMPCCAdvertisedDeviceCallbacks( CameraMap_t & _Cameras,
                                                                BLEUUID & _cameraControlServiceUUID )
  : BLEAdvertisedDeviceCallbacks()
  , m_Cameras( _Cameras )
  , m_cameraControlServiceUUID( _cameraControlServiceUUID )
{
}

BMPCCAdvertisedDeviceCallbacks::~BMPCCAdvertisedDeviceCallbacks()
{
}


void BMPCCAdvertisedDeviceCallbacks::onResult( BLEAdvertisedDevice advertisedDevice )
{
  Serial.print("BLE Advertised Device found: ");
  Serial.println(advertisedDevice.toString().c_str());
  // We have found a device, let us now see if it contains the service we are looking for.
  if (advertisedDevice.haveServiceUUID()
      && advertisedDevice.isAdvertisingService( m_cameraControlServiceUUID ))
  {
    Serial.println("found a camera.");
    BLEAddress address( advertisedDevice.getAddress() );
    std::string addressStr( address.toString() );
    std::string msg = "Camera address: " + addressStr;
    Serial.println( msg.c_str() );
    if( m_Cameras.end() == m_Cameras.find( addressStr ) )
    {
      m_Cameras[ addressStr ] = new BLEAdvertisedDevice( advertisedDevice );
    }
  }
}
