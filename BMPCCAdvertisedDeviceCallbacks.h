#ifndef BMPCCADVERTISEDDEVICECALLBACKS_H
#define BMPCCADVERTISEDDEVICECALLBACKS_H

#include <BLEAdvertisedDevice.h>

typedef std::map< std::string, BLEAdvertisedDevice * > CameraMap_t;

/**
 * For each advertised device, check to see if it is a BMPCC camera.
 */
class BMPCCAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
  public:
    BMPCCAdvertisedDeviceCallbacks( CameraMap_t & _Cameras,
                                    BLEUUID & _cameraControlServiceUUID );
    virtual ~BMPCCAdvertisedDeviceCallbacks();

  protected:
    virtual void onResult( BLEAdvertisedDevice advertisedDevice );

  private:
    CameraMap_t & m_Cameras;
    BLEUUID & m_cameraControlServiceUUID;

};


#endif
